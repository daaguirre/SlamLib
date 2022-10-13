
#include <random>

#include "odometry/odometry_models.h"
#include "particle_filter.h"
#include "slam_lib/utils/math_helpers.h"

namespace slam
{

template <typename FloatT>
ParticleFilter<FloatT>::ParticleFilter(
    const size_t num_particles,
    typename OccupancyGrid<FloatT>::ConstPtr occ_grid)
    : m_num_particles(num_particles), m_occ_grid(occ_grid), m_generator(std::random_device()())
{
    init_particles();
}

template <typename FloatT>
void ParticleFilter<FloatT>::init_particles()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    m_num_particles_inv = 1.0 / static_cast<FloatT>(m_num_particles);
    m_particles.resize(m_num_particles);

    size_t count = 0;
    while (count < m_num_particles)
    {
        Particle& particle = m_particles[count];
        std::uniform_real_distribution<FloatT> dis(0, 1);  // uniform distribution between 0 and 1
        particle.x = int(dis(gen) * m_occ_grid->width());
        particle.y = int(dis(gen) * m_occ_grid->height());
        CellState cell_state = m_occ_grid->check_cell_state(particle, 0.1, 0.95);
        // make sure particles are initialized in free cells
        if (cell_state != CellState::FREE)
        {
            continue;
        }

        // yaw in [-pi, pi] range
        particle.yaw = wrap_to_pi_range(dis(gen) * 2 * pi);
        particle.weight = m_num_particles_inv;
        ++count;
    }
}

template <typename FloatT>
typename ParticleFilter<FloatT>::Particle ParticleFilter<FloatT>::update(
    const LidarOdometry<FloatT>& previous_odometry,
    const LidarOdometry<FloatT>& current_odometry,
    const std::vector<FloatT>& scan_angles)
{
    FloatT total_weight = 0;
    std::vector<Particle> particles_tmp;
    particles_tmp.reserve(m_num_particles);
    for (size_t i = 0; i < m_num_particles; i++)
    {
        // calculate new state
        Particle current_particle;
        *dynamic_cast<GridPose*>(&current_particle) =
            sample_motion_model(m_particles[i], previous_odometry, current_odometry);
        CellState cell_state = m_occ_grid->check_cell_state(current_particle);
        FloatT weight = m_particles[i].weight;
        if(cell_state != CellState::FREE)
        {
            weight = 0;
        }
        // FloatT weight = cell_state == CellState::FREE ? m_particles[i].weight : 0;
        if (current_odometry.ranges.size() > 0 && weight > 0)
        {
            weight = sample_measurement_model(current_particle, current_odometry, scan_angles);
        }

        current_particle.weight = weight;
        particles_tmp.push_back(current_particle);
        total_weight += weight;
    }

    // normalize weights
    std::for_each(
        particles_tmp.begin(),
        particles_tmp.end(),
        [=](Particle& particle) { particle.weight /= total_weight; });

    auto best_particle = std::max_element(
        particles_tmp.begin(),
        particles_tmp.end(),
        [](const Particle& a, const Particle& b) { return a.weight < b.weight; });

    Particle final_particle = *best_particle;

    FloatT sum1 = 0, sum2 = 0;
    for(const auto& p : particles_tmp)
    {
        sum1 += p.weight;
        sum2 += p.weight * p.weight;
    }

    FloatT eff_particles = ((sum1*sum1) / sum2);
    FloatT sampler_thr = m_num_particles * 0.8; 
    if (eff_particles < sampler_thr)
    {
        m_particles = low_variance_sampler(particles_tmp);
    }
    else
    {
        m_particles = particles_tmp;
    }
    
    return final_particle;
}

template <typename FloatT>
typename ParticleFilter<FloatT>::GridPose ParticleFilter<FloatT>::sample_motion_model(
    const GridPose& previous_map_pose,
    const LidarOdometry<FloatT>& previous_odometry,
    const LidarOdometry<FloatT>& current_odometry)
{
    FloatT delta_x = current_odometry.x - previous_odometry.x;
    FloatT delta_y = current_odometry.y - previous_odometry.y;
    FloatT delta_yaw = current_odometry.yaw - previous_odometry.yaw;
    FloatT delta_t = sqrt(delta_x * delta_x + delta_y * delta_y);
    FloatT theta = atan2(delta_y, delta_x);
    FloatT delta_rot1 = theta - previous_odometry.yaw;
    FloatT delta_rot2 = current_odometry.yaw - theta;

    FloatT motion_noise_sigma = m_alpha3 * delta_t + m_alpha4 * (delta_rot1 + delta_rot2);
    std::normal_distribution<FloatT> translation_noise(0.0, motion_noise_sigma);

    FloatT rot1_noise_sigma = std::abs(m_alpha1 * delta_rot1 + m_alpha2 * delta_t);
    std::normal_distribution<FloatT> rot1_noise(0.0, rot1_noise_sigma);

    FloatT rot2_noise_sigma = std::abs(m_alpha1 * delta_rot2 + m_alpha2 * delta_t);
    std::normal_distribution<FloatT> rot2_noise(0.0, rot2_noise_sigma);

    FloatT delta_t_hat = delta_t - translation_noise(m_generator);
    FloatT delta_rot1_hat = delta_rot1 - rot1_noise(m_generator);
    FloatT delta_rot2_hat = delta_rot2 - rot2_noise(m_generator);

    Pose previous_pose = m_occ_grid->to_world_frame(previous_map_pose);
    Pose new_pose;
    FloatT dx = (delta_t_hat * cos(previous_pose.yaw + delta_rot1_hat));
    FloatT dy = (delta_t_hat * sin(previous_pose.yaw + delta_rot1_hat));
    new_pose.x = previous_pose.x + dx;
    new_pose.y = previous_pose.y + dy;
    new_pose.yaw = previous_pose.yaw + delta_rot1_hat + delta_rot2_hat;

    GridPose new_map_pose = m_occ_grid->to_map_frame(new_pose);
    return new_map_pose;
}

template <typename FloatT>
FloatT ParticleFilter<FloatT>::sample_measurement_model(
    const Particle& robot_pose,
    const LidarOdometry<FloatT>& current_odometry,
    const std::vector<FloatT>& scan_angles)
{
    Particle lidar_pose;
    *dynamic_cast<Point*>(&lidar_pose) = m_occ_grid->project_ray(robot_pose, m_lidar_offset);
    lidar_pose.yaw = robot_pose.yaw;
    if (m_occ_grid->check_cell_state(lidar_pose) != CellState::FREE)
    {
        return 0;
    }

    FloatT score = 0;
    for (int i = 0; i < scan_angles.size(); ++i)
    {
        const FloatT range = current_odometry.ranges[i];
        if (range > m_max_range)
        {
            // laser reading is not valid if exceeds max range
            continue;
        }

        const FloatT scan_angle = scan_angles[i];
        Particle scan_pose(lidar_pose);
        scan_pose.yaw = wrap_to_pi_range(scan_pose.yaw - scan_angle);
        // FloatT max_range = m_occ_grid->ray_tracing(scan_pose);
        // max_range = std::isnan(max_range)? 0 : max_range*1.2;
        // if(range > max_range)
        // {
        //     continue;
        // }

        Point laser_end = m_occ_grid->project_ray(scan_pose, range);
        CellState cell_state = m_occ_grid->check_cell_state(laser_end, 0.15);
        score += cell_state == CellState::OCCUPIED ? 5 : -2;
    }

    if(score < 0)
    {
        score = 0; 
    }

    return score;
}

template <typename FloatT>
FloatT ParticleFilter<FloatT>::sample_hit_model(const Particle& state, const FloatT range)
{
    FloatT true_range = m_occ_grid->ray_tracing(state);
    std::normal_distribution<FloatT> hit_distribution(true_range, m_sigma_hit);
    FloatT hit_prob = 0;  // hit_distribution(range);
    return hit_prob;
}

template <typename FloatT>
std::vector<typename ParticleFilter<FloatT>::Particle> ParticleFilter<FloatT>::low_variance_sampler(
    const std::vector<typename ParticleFilter<FloatT>::Particle>& particle_set)
{
    std::vector<Particle> particles_tmp;
    float r = (rand() / static_cast<FloatT>(RAND_MAX)) * m_num_particles_inv;
    float c = particle_set[0].weight;
    size_t i = 0;
    for (size_t m = 0; m < m_num_particles; ++m)
    {
        float u = r + static_cast<FloatT>(m) * m_num_particles_inv;
        while (u > c)
        {
            i++;
            c += particle_set[i].weight;
        }
        particles_tmp.push_back(particle_set[i]);
    }

    return particles_tmp;
}

}  // namespace slam
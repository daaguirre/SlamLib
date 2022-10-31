
#include <iostream>
#include <random>
#include <omp.h>
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
    int num_threads = omp_get_max_threads();
    omp_set_num_threads(num_threads);
    std::cout << "OpenMP initialized with " << num_threads << " threads.\n";

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
        CellState cell_state = m_occ_grid->check_cell_state(particle);
        // make sure particles are initialized in free cells
        if (cell_state != CellState::FREE)
        {
            continue;
        }

        // yaw in [-pi, pi] range
        particle.yaw = wrap_to_pi_range(dis(gen) * 2 * M_PI);
        particle.weight = m_num_particles_inv;
        ++count;
    }
}

template <typename FloatT>
typename ParticleFilter<FloatT>::Particle ParticleFilter<FloatT>::update(
    const RobotReading<FloatT>& previous_reading,
    const RobotReading<FloatT>& current_reading)
{
    FloatT total_weight = 0;

    #pragma omp parallel for reduction(+: total_weight)
    for (size_t i = 0; i < m_num_particles; i++)
    {
        // calculate new state
        Particle current_particle;
        *dynamic_cast<IPose<FloatT>*>(&current_particle) =
            sample_motion_model(m_particles[i], previous_reading, current_reading);
        CellState cell_state = m_occ_grid->check_cell_state(current_particle);
        FloatT weight = m_particles[i].weight;
        if (cell_state != CellState::FREE)
        {
            weight = 0;
        }
        if (current_reading.ranges.size() > 0 && weight > 0)
        {
            weight = sample_measurement_model(current_particle, current_reading);
        }

        current_particle.weight = weight;
        m_particles[i] = current_particle;
        total_weight += weight;
    }

    // normalize weights
    std::for_each(
        m_particles.begin(),
        m_particles.end(),
        [=](Particle& p) { p.weight /= total_weight; });

    FloatT x=0, y=0, yaw=0;
    FloatT sum1 = 0, sum2 = 0;
    for(const auto& p : m_particles)
    {
        x += p.x * p.weight;
        y += p.y * p.weight;
        yaw += p.yaw * p.weight;
        sum1 += p.weight;
        sum2 += p.weight * p.weight;
    }
    Particle best_particle;
    best_particle.x = std::round(x);
    best_particle.y = std::round(y);
    best_particle.yaw = yaw;

    FloatT eff_particles = ((sum1 * sum1) / sum2) / m_num_particles;
    if (eff_particles < m_resampling_thr)
    {
        m_particles = low_variance_sampler(m_particles);
    }

    return best_particle;
}

template <typename FloatT>
IPose<FloatT> ParticleFilter<FloatT>::sample_motion_model(
    const IPose<FloatT>& previous_map_pose,
    const RobotReading<FloatT>& previous_reading,
    const RobotReading<FloatT>& current_reading)
{
    FloatT delta_x = current_reading.x - previous_reading.x;
    FloatT delta_y = current_reading.y - previous_reading.y;
    FloatT delta_yaw = current_reading.yaw - previous_reading.yaw;
    FloatT delta_t = sqrt(delta_x * delta_x + delta_y * delta_y);
    FloatT theta = atan2(delta_y, delta_x);
    FloatT delta_rot1 = theta - previous_reading.yaw;
    FloatT delta_rot2 = current_reading.yaw - theta;

    FloatT motion_noise_sigma = m_alpha3 * delta_t + m_alpha4 * (delta_rot1 + delta_rot2);
    std::normal_distribution<FloatT> translation_noise(0.0, motion_noise_sigma);

    FloatT rot1_noise_sigma = std::abs(m_alpha1 * delta_rot1 + m_alpha2 * delta_t);
    std::normal_distribution<FloatT> rot1_noise(0.0, rot1_noise_sigma);

    FloatT rot2_noise_sigma = std::abs(m_alpha1 * delta_rot2 + m_alpha2 * delta_t);
    std::normal_distribution<FloatT> rot2_noise(0.0, rot2_noise_sigma);

    FloatT delta_t_hat = delta_t - translation_noise(m_generator);
    FloatT delta_rot1_hat = delta_rot1 - rot1_noise(m_generator);
    FloatT delta_rot2_hat = delta_rot2 - rot2_noise(m_generator);

    Pose<FloatT> previous_pose = m_occ_grid->to_world_frame(previous_map_pose);
    Pose<FloatT> new_pose;
    FloatT dx = (delta_t_hat * cos(previous_pose.yaw + delta_rot1_hat));
    FloatT dy = (delta_t_hat * sin(previous_pose.yaw + delta_rot1_hat));
    new_pose.x = previous_pose.x + dx;
    new_pose.y = previous_pose.y + dy;
    new_pose.yaw = previous_pose.yaw + delta_rot1_hat + delta_rot2_hat;

    IPose<FloatT> new_map_pose = m_occ_grid->to_map_frame(new_pose);
    return new_map_pose;
}

template <typename FloatT>
FloatT ParticleFilter<FloatT>::sample_measurement_model(
    const Particle& robot_pose,
    const RobotReading<FloatT>& current_reading)
{
    Particle lidar_pose;
    *dynamic_cast<IPoint*>(&lidar_pose) =
        m_occ_grid->project_ray(robot_pose, current_reading.lidar_cfg_ptr->pos_offset());
    lidar_pose.yaw = robot_pose.yaw;
    if (m_occ_grid->check_cell_state(lidar_pose) != CellState::FREE)
    {
        return 0;
    }

    const auto& scan_angles = current_reading.lidar_cfg_ptr->scan_angles();
    FloatT score = 0;

    #pragma omp parallel for reduction(+: score)
    for (int i = 0; i < scan_angles.size(); ++i)
    {
        const FloatT range = current_reading.ranges[i];
        if (range > current_reading.lidar_cfg_ptr->max_range())
        {
            // laser reading is not valid if exceeds max range
            continue;
        }

        const FloatT scan_angle = scan_angles[i];
        Particle scan_pose(lidar_pose);
        scan_pose.yaw = wrap_to_pi_range(scan_pose.yaw - scan_angle);

        FloatT hit_prob = sample_hit_model(scan_pose, range);
        score += hit_prob;

        // IPoint laser_end = m_occ_grid->project_ray(scan_pose, range);
        // CellState cell_state = m_occ_grid->check_cell_state(laser_end);
        // score += (cell_state == CellState::OCCUPIED ? 1 : -0) * m_num_particles_inv;
    }

    if (score < 0)
    {
        score = 0;
    }

    return score;
}

template <typename FloatT>
FloatT ParticleFilter<FloatT>::sample_hit_model(const Particle& particle, const FloatT range)
{
    FloatT true_range = m_occ_grid->ray_tracing(particle, 0.15, 0.15);
    if (std::isnan(true_range))
    {
        return 0;
    }

    FloatT hit_prob = normpdf(range, true_range, m_sigma_hit);
    // FloatT max_hit_prob = normpdf(true_range, true_range, m_sigma_hit);
    // hit_prob /= max_hit_prob;
    return hit_prob;
}

template <typename FloatT>
std::vector<typename ParticleFilter<FloatT>::Particle> ParticleFilter<FloatT>::low_variance_sampler(
    const std::vector<typename ParticleFilter<FloatT>::Particle>& particle_set)
{
    std::vector<Particle> particles_tmp;
    particles_tmp.reserve(m_num_particles);
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

#include <random>

#include "odometry/odometry_models.h"
#include "particle_filter.h"

namespace slam
{

template <typename FloatT>
ParticleFilter<FloatT>::ParticleFilter(
    const size_t num_particles,
    typename OccupancyGrid<FloatT>::ConstPtr occ_grid)
    : m_num_particles(num_particles), m_occ_grid(occ_grid)
{
    init_particles();
}

template <typename FloatT>
void ParticleFilter<FloatT>::init_particles()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    FloatT initial_weight = 1.0 / static_cast<FloatT>(m_num_particles);
    m_weights = Vector<FloatT>::Constant(m_num_particles, initial_weight);
    m_states.resize(m_num_particles);
    size_t count = m_num_particles - 1;
    while (count)
    {
        Pose state;
        std::uniform_real_distribution<FloatT> dis(0, 1);  // uniform distribution between 0 and 1
        state.x = int(dis(gen) * m_occ_grid->width());
        state.y = int(dis(gen) * m_occ_grid->height());
        FloatT occ_prob = m_occ_grid->map()(state.y, state.x);
        // make sure particles are initialized in free cells
        if (occ_prob < 0.5 && occ_prob >= 0)
        {
            continue;
        }

        state.yaw = dis(gen) * 2 * pi;
        // yaw in [-pi, pi] range
        state.yaw = state.yaw > pi ? state.yaw - 2 * pi : state.yaw;
        m_states[count--] = state;
    }
}

template <typename FloatT>
typename ParticleFilter<FloatT>::Pose ParticleFilter<FloatT>::update(
    const LidarOdometry<FloatT>& previous_odometry,
    const LidarOdometry<FloatT>& current_odometry,
    const Vector<FloatT>& scan_angles)
{
    for (size_t i = 0; i < m_num_particles; i++)
    {
        // calculate new state
        Pose current_state = sample_motion_model(m_states[i], previous_odometry, current_odometry);
    }

    return Pose();
}

template <typename FloatT>
typename ParticleFilter<FloatT>::Pose ParticleFilter<FloatT>::sample_motion_model(
    const Pose& previous_state,
    const LidarOdometry<FloatT>& previous_odometry,
    const LidarOdometry<FloatT>& current_odometry)
{
    FloatT delta_x = current_odometry.x - previous_odometry.x;
    FloatT delta_y = current_odometry.y - previous_odometry.y;
    FloatT delta_yaw = current_odometry.yaw - previous_odometry.yaw;
    FloatT delta_t = sqrt(delta_x * delta_x + delta_y * delta_y);
    FloatT theta = atan2(delta_y, delta_x);
    FloatT delta_rot1 = atan2(delta_y, delta_x) - previous_odometry.yaw;
    FloatT delta_rot2 = current_odometry.yaw - theta;

    FloatT motion_noise_sigma = m_alpha3 * delta_t + m_alpha4 * (delta_rot1 + delta_rot2);
    std::normal_distribution<FloatT> translation_noise(0.0, motion_noise_sigma);

    FloatT rot1_noise_sigma = m_alpha1 * delta_rot1 + m_alpha2 * delta_t;
    std::normal_distribution<FloatT> rot1_noise(0.0, rot1_noise_sigma);

    FloatT rot2_noise_sigma = m_alpha1 * delta_rot2 + m_alpha2 * delta_t;
    std::normal_distribution<FloatT> rot2_noise(0.0, rot2_noise_sigma);

    FloatT delta_t_hat = delta_t - translation_noise(m_generator);
    FloatT delta_rot1_hat = delta_rot1 - rot1_noise(m_generator);
    FloatT delta_rot2_hat = delta_rot2 - rot2_noise(m_generator);

    Pose new_state;
    FloatT dx = (delta_t_hat * cos(previous_state.yaw + delta_rot1_hat)) / m_occ_grid->resolution();
    FloatT dy = (delta_t_hat * sin(previous_state.yaw + delta_rot1_hat)) / m_occ_grid->resolution();
    new_state.x = previous_state.x + dx;
    new_state.y = previous_state.y + dy;
    new_state.yaw = previous_state.yaw + delta_rot1_hat + delta_rot2_hat;

    return new_state;
}

template <typename FloatT>
FloatT ParticleFilter<FloatT>::sample_measurement_model(
    const Pose& robot_pose,
    const LidarOdometry<FloatT>& current_odometry,
    const Vector<FloatT>& scan_angles)
{
    Pose lidar_pose;
    lidar_pose.x = static_cast<int>(round(robot_pose.x + m_lidar_offset * cos(robot_pose.yaw)));
    lidar_pose.y = static_cast<int>(round(robot_pose.y + m_lidar_offset * sin(robot_pose.yaw)));
    lidar_pose.yaw = robot_pose.yaw;
    for (int i = 0; i < scan_angles.rows(); ++i)
    {
        FloatT scan_angle = scan_angles(i);

    }

    return 0;
}

template <typename FloatT>
std::tuple<int, int> ParticleFilter<FloatT>::calculate_pos_from_range(
    const Pose& pose,
    const FloatT range,
    const FloatT theta)
{
    int x = pose.x + round(range * cos(theta));
    int y = pose.y + round(range * sin(theta));
    return {x,y};
}

template <typename FloatT>
FloatT ParticleFilter<FloatT>::ray_tracing(const Pose& pose, const FloatT theta)
{
    FloatT step = 1.0;
    FloatT range = -1;
    int beam_x = pose.x;
    int beam_y = pose.y;
    while (beam_x < m_occ_grid->width() && beam_y < m_occ_grid->height())
    {
        beam_x += int(round(step * cos(theta)));
        beam_y += int(round(step * sin(theta)));
        FloatT occ_prob = m_occ_grid->map()(beam_y, beam_x);
        if(occ_prob < 0)
        {
            break;
        }
        if(occ_prob >= m_obstacle_th)
        {
            range = sqrt(pow(pose.x - beam_x, 2) + pow(pose.y - beam_y, 2)) * m_occ_grid->resolution();
            break;
        }
    }
    return range;
}

}  // namespace slam
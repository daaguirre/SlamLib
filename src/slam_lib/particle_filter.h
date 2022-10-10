/**
 * @file particle_filter.h
 * @author daaguirre
 * @brief
 * @date 2022-08-20
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __SLAM_LIB_PARTICLE_FILTER_H_
#define __SLAM_LIB_PARTICLE_FILTER_H_

#include "mapping/occupancy_grid.h"
#include "odometry/odometry_models.h"

namespace slam
{

static constexpr double pi = 3.14159265358979323846;

template <typename FloatT>
class ParticleFilter
{
    FloatT MU_NOISE = 0;
    FloatT SIGMA_MOTION_NOISE = 0.1;
    FloatT SIGMA_YAW_NOISE = 0.01;

public:
    struct Pose
    {
        int x;
        int y;
        FloatT yaw;
    };

    ParticleFilter(const size_t num_particles, typename OccupancyGrid<FloatT>::ConstPtr occ_grid);

    Pose update(
        const LidarOdometry<FloatT>& current_odometry,
        const LidarOdometry<FloatT>& previous_odometry,
        const Vector<FloatT>& scan_angles);

    const std::vector<Pose>& get_particles() const
    {
        return m_states;
    }

private:
    void init_particles();
    Pose sample_motion_model(
        const Pose& previous_state,
        const LidarOdometry<FloatT>& previous_odometry,
        const LidarOdometry<FloatT>& current_odometry);

    FloatT sample_measurement_model(
        const Pose& pose,
        const LidarOdometry<FloatT>& current_odometry,
        const Vector<FloatT>& scan_angles);

    std::tuple<int, int> calculate_pos_from_range(const Pose& pose, const FloatT range, const FloatT theta);
    FloatT ray_tracing(const Pose& pose, const FloatT theta);

    const size_t m_num_particles;
    typename OccupancyGrid<FloatT>::ConstPtr m_occ_grid;
    Vector<FloatT> m_weights;
    std::vector<Pose> m_states;

    std::default_random_engine m_generator;

    // algorithm paramters
    FloatT m_alpha1{0.025};
    FloatT m_alpha2{0.025};
    FloatT m_alpha3{0.4};
    FloatT m_alpha4{0.4};
    FloatT m_sigma_hit{2};
    FloatT m_lambda_short{1.5};
    FloatT m_z_hit{0.8};
    FloatT m_z_short{0.2};
    FloatT m_z_rand{0.0};
    FloatT m_z_max{0.0};
    FloatT m_lidar_offset{0.25};
    FloatT m_obstacle_th{0.6};
};

}  // namespace slam

#endif  // __SLAM_LIB_PARTICLE_FILTER_H_

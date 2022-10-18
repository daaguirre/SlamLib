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

#include <random>

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
    using GridPose = typename OccupancyGrid<FloatT>::GridPose;
    using Point = typename OccupancyGrid<FloatT>::Point;
    using Pose = typename OccupancyGrid<FloatT>::Pose;

    struct Particle : public GridPose
    {
        FloatT weight;
    };

    ParticleFilter(const size_t num_particles, typename OccupancyGrid<FloatT>::ConstPtr occ_grid);

    Particle update(
        const LidarOdometry<FloatT>& previous_odometry,
        const LidarOdometry<FloatT>& current_odometry,
        const std::vector<FloatT>& scan_angles);

    const std::vector<Particle>& get_particles() const
    {
        return m_particles;
    }

private:
    void init_particles();
    GridPose sample_motion_model(
        const GridPose& previous_map_pose,
        const LidarOdometry<FloatT>& previous_odometry,
        const LidarOdometry<FloatT>& current_odometry);

    FloatT sample_measurement_model(
        const Particle& pose,
        const LidarOdometry<FloatT>& current_odometry,
        const std::vector<FloatT>& scan_angles);

    FloatT sample_hit_model(const Particle& particle, const FloatT range);
    std::vector<Particle> low_variance_sampler(const std::vector<Particle>& particle_set);

    const size_t m_num_particles;
    typename OccupancyGrid<FloatT>::ConstPtr m_occ_grid;
    std::vector<Particle> m_particles;

    std::mt19937 m_generator;

    // algorithm paramters
    FloatT m_num_particles_inv;
    FloatT m_alpha1{0.025};
    FloatT m_alpha2{0.025};
    FloatT m_alpha3{0.4};
    FloatT m_alpha4{0.4};
    FloatT m_sigma_hit{0.4};
    FloatT m_lambda_short{1.5};
    FloatT m_z_hit{0.8};
    FloatT m_z_short{0.2};
    FloatT m_z_rand{0.0};
    FloatT m_z_max{0.0};
    FloatT m_lidar_offset{0.25};
    FloatT m_obstacle_th{0.6};
    FloatT m_max_range{114};
};

}  // namespace slam

#endif  // __SLAM_LIB_PARTICLE_FILTER_H_

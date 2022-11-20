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

#include "particle_filter_base.h"
#include "slam_lib/mapping/occupancy_grid.h"
#include "slam_lib/odometry/odometry_models.h"

namespace slam
{

template <typename FloatT>
class ParticleFilter : public ParticleFilterBase<FloatT>
{
public:
    using Particle = typename ParticleFilterBase<FloatT>::Particle;

    ParticleFilter(const size_t num_particles, typename OccupancyGrid<FloatT>::ConstPtr occ_grid);

    virtual Particle update(
        const RobotReading<FloatT>& previous_odometry,
        const RobotReading<FloatT>& current_odometry) override;

    void set_resampling_thr(const FloatT resampling_thr)
    {
        m_resampling_thr = resampling_thr;
    }

private:
    virtual void init_particles() override;

    IPose<FloatT> sample_motion_model(
        const IPose<FloatT>& previous_map_pose,
        const RobotReading<FloatT>& previous_reading,
        const RobotReading<FloatT>& current_reading);

    FloatT sample_measurement_model(
        const Particle& pose,
        const RobotReading<FloatT>& current_reading);

    FloatT sample_hit_model(const Particle& particle, const FloatT range);
    std::vector<Particle> low_variance_sampler(const std::vector<Particle>& particle_set);

    std::mt19937 m_generator;

    // algorithm parameters
    FloatT m_num_particles_inv;
    FloatT m_alpha1{0.025};
    FloatT m_alpha2{0.025};
    FloatT m_alpha3{0.4};
    FloatT m_alpha4{0.4};
    FloatT m_sigma_hit{0.8};
    FloatT m_resampling_thr{1.0};
};

}  // namespace slam

#endif  // __SLAM_LIB_PARTICLE_FILTER_H_

/**
 * @file particle_filter_base.h
 * @author daaguirre
 * @brief
 * @date 2022-08-20
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __SLAM_LIB_LOCALIZATION_PARTICLE_FILTER_BASE_H_
#define __SLAM_LIB_LOCALIZATION_PARTICLE_FILTER_BASE_H_

#include <random>

#include "slam_lib/mapping/occupancy_grid.h"
#include "slam_lib/odometry/odometry_models.h"

namespace slam
{

template <typename FloatT>
class ParticleFilterBase
{
public:
    struct Particle : public IPose<FloatT>
    {
        FloatT weight{0};
    };

    ParticleFilterBase(
        const size_t num_particles,
        typename OccupancyGrid<FloatT>::ConstPtr occ_grid) noexcept
        : m_num_particles(num_particles), m_occ_grid(occ_grid)
    {
    }

    virtual ~ParticleFilterBase() noexcept {}

    virtual Particle update(
        const RobotReading<FloatT>& previous_odometry,
        const RobotReading<FloatT>& current_odometry) = 0;

    const std::vector<Particle>& get_particles() const
    {
        return m_particles;
    }

protected:
    virtual void init_particles() = 0;

    const size_t m_num_particles;
    typename OccupancyGrid<FloatT>::ConstPtr m_occ_grid;
    std::vector<Particle> m_particles;
};

}  // namespace slam

#endif  // __SLAM_LIB_LOCALIZATION_PARTICLE_FILTER_BASE_H_

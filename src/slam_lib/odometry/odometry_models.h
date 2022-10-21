/**
 * @file robot_odometry.h
 * @author daaguirre
 * @brief
 * @date 2022-08-20
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __SLAM_LIB_ODOMETRY_ROBOT_ODOMETRY_H_
#define __SLAM_LIB_ODOMETRY_ROBOT_ODOMETRY_H_

#include <Eigen/Eigen>
#include <memory>

#include "slam_lib/types.h"

namespace slam
{

template <typename FloatT>
using Vector = Eigen::Matrix<FloatT, -1, 1>;

template <typename FloatT>
struct PositionReading
{
    FloatT x;
    FloatT y;
    FloatT yaw;
};

template <typename FloatT>
class LidarConfigBuilder;

template <typename FloatT>
class LidarConfig
{
public:
    using Ptr = std::shared_ptr<LidarConfig<FloatT>>;
    using ConstPtr = std::shared_ptr<const LidarConfig<FloatT>>;

    size_t num_scans() const
    {
        return m_scan_angles.size();
    }

    const FloatT min_angle() const
    {
        return m_scan_angles.front();
    }

    const FloatT max_angle() const
    {
        return m_scan_angles.back();
    }

    FloatT max_range() const
    {
        return m_max_range;
    }

    const std::string& name() const
    {
        return m_name;
    }

    const std::string& frame_name() const
    {
        return m_frame_name;
    }

    FloatT step() const
    {
        return m_step;
    }

    const std::vector<FloatT>& scan_angles() const
    {
        return m_scan_angles;
    }

    FloatT pos_offset() const
    {
        return m_pos_offset;
    }

    friend class LidarConfigBuilder<FloatT>;

private:
    LidarConfig(const std::string& name) : m_name(name), m_frame_name(name + "_frame") {}

    std::string m_name;
    std::string m_frame_name;
    FloatT m_max_range{1000};
    FloatT m_step;
    std::vector<FloatT> m_scan_angles;
    FloatT m_pos_offset;  // lidar position offset wrt the robot center
};

template <typename FloatT>
class LidarConfigBuilder
{
public:
    LidarConfigBuilder(const std::string& name) : m_lidar_cfg(name) {}

    LidarConfigBuilder& set_max_range(const FloatT max_range)
    {
        m_lidar_cfg.m_max_range = max_range;
        return *this;
    }

    LidarConfigBuilder& set_position_offset(const FloatT offset)
    {
        m_lidar_cfg.m_pos_offset = offset;
        return *this;
    }

    LidarConfigBuilder& set_scan_range(
        const size_t num_scans,
        const FloatT min_angle,
        const FloatT max_angle)
    {
        FloatT scan_range = max_angle - min_angle;
        FloatT step = scan_range / num_scans;
        m_lidar_cfg.m_scan_angles.reserve(num_scans);
        for (size_t i = 0; i < num_scans; ++i)
        {
            FloatT angle = i * step + min_angle;
            m_lidar_cfg.m_scan_angles[i] = angle;
        }

        m_lidar_cfg.m_step = step;

        return *this;
    }

    LidarConfig<FloatT> build() const
    {
        if (m_lidar_cfg.m_scan_angles.empty())
        {
            throw std::runtime_error(
                "Please at least set scan range for lidar " + m_lidar_cfg.m_name);
        }
        return std::move(m_lidar_cfg);
    }

    typename LidarConfig<FloatT>::Ptr build_as_shared() const
    {
        return std::make_shared<LidarConfig<FloatT>>(std::move(build()));
    }

private:
    LidarConfig<FloatT> m_lidar_cfg;
};

/**
 * @brief Represents the odometry of a robot for a
 * a single position.
 */
template <typename FloatT>
struct LidarReading
{
    FloatT lx;
    FloatT ly;
    FloatT lyaw;
    Vector<FloatT> ranges;
    typename LidarConfig<FloatT>::ConstPtr lidar_cfg_ptr;
};

template <typename FloatT>
struct RobotReading : public PositionReading<FloatT>, public LidarReading<FloatT>
{
    FloatT timestamp;
};

template <typename FloatT>
using RobotOdometry = std::vector<RobotReading<FloatT>>;


}  // namespace slam

#endif  // __SLAM_LIB_ODOMETRY_ROBOT_ODOMETRY_H_

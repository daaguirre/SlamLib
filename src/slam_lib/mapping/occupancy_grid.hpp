
#include <assert.h>

#include <Eigen/Eigen>
#include <cmath>

#include "occupancy_grid.h"
#include "rclcpp/rclcpp.hpp"
#include "slam_lib/utils/math_helpers.h"

namespace slam
{

template <typename FloatT>
const IPoint OccupancyGrid<FloatT>::INVALID_POSITION = {-1, -1};

template <typename FloatT>
const FloatT OccupancyGrid<FloatT>::INVALID_RANGE = std::numeric_limits<FloatT>::quiet_NaN();

template <typename FloatT>
OccupancyGrid<FloatT>::OccupancyGrid(
    const int width,
    const int height,
    const FloatT resolution,
    const std::string &name)
    : m_resolution(resolution), m_name(name)
{
    m_grid_center = {width / 2, height / 2};
    m_map.resize(height, width);
    m_map.setConstant(0.5);
    FloatT inv_res = 1 / m_resolution;
    Eigen::Matrix<FloatT, 2, 2> rot_mat;
    rot_mat << 1, 0, 0, -1;
    m_transform = rot_mat * Eigen::Translation<FloatT, 2>(0, -height) * Eigen::Scaling(inv_res);
}

template <typename FloatT>
IPose<FloatT> OccupancyGrid<FloatT>::to_map_frame(const Pose<FloatT> &pose2d) const
{
    IPose<FloatT> grid_pose;
    auto vector = (m_transform * pose2d.vector());
    vector(0) = std::round(vector(0));
    vector(1) = std::round(vector(1));
    grid_pose.vector() = vector.template cast<int>();
    grid_pose.yaw = -pose2d.yaw;
    return grid_pose;
}

template <typename FloatT>
Pose<FloatT> OccupancyGrid<FloatT>::to_world_frame(const IPose<FloatT> &pose) const
{
    Pose<FloatT> world_pose;
    world_pose.vector() = m_transform.inverse() * pose.vector().template cast<FloatT>();
    world_pose.yaw = -pose.yaw;
    return world_pose;
}

template <typename FloatT>
IPoint OccupancyGrid<FloatT>::project_ray(const IPose<FloatT> &pose, const FloatT range) const
{
    FloatT dx = range * std::cos(pose.yaw);
    FloatT dy = range * std::sin(pose.yaw);
    int new_x = pose.x + std::round(dx / m_resolution);
    int new_y = pose.y + std::round(dy / m_resolution);
    return {new_x, new_y};
}

template <typename FloatT>
CellState OccupancyGrid<FloatT>::check_cell_state(
    const IPose<FloatT> &pose,
    const FloatT occ_thr,
    const FloatT free_thr) const
{
    const IPoint *point_ptr = dynamic_cast<const IPoint *>(&pose);
    return check_cell_state(*point_ptr);
}

template <typename FloatT>
CellState OccupancyGrid<FloatT>::check_cell_state(
    const IPoint &point,
    const FloatT occ_thr,
    const FloatT free_thr) const

{
    if (point.x < width() && point.x >= 0 && point.y < height() && point.y >= 0)
    {
        FloatT occ_prob = m_map(point.y, point.x);
        if (occ_prob < occ_thr && occ_prob >= 0)
        {
            return CellState::OCCUPIED;
        }
        else if (occ_prob >= free_thr && occ_prob <= 1.0)
        {
            return CellState::FREE;
        }
        return CellState::UNKNOWN;
    }

    return CellState::OUT_OF_BOUNDS;
}

template <typename FloatT>
FloatT OccupancyGrid<FloatT>::ray_tracing_in_world_frame(
    const IPose<FloatT> &pose,
    const FloatT occ_thr,
    const FloatT free_thr) const
{
    IPoint current_point(pose.x, pose.y);
    CellState current_state = check_cell_state(current_point, occ_thr, free_thr);
    FloatT range = current_state == CellState::FREE ? 0 : INVALID_RANGE;
    int counter = 1;
    while (current_state == CellState::FREE)
    {
        range = m_resolution * counter++;
        current_point = project_ray(pose, range);
        current_state = check_cell_state(current_point, occ_thr, free_thr);
        if (current_state >= CellState::UNKNOWN)
        {
            range = INVALID_RANGE;
        }
    }
    return range;
}

template <typename FloatT>
FloatT OccupancyGrid<FloatT>::ray_tracing(
    const IPose<FloatT> &pose,
    const FloatT occ_thr,
    const FloatT free_thr) const
{
    FloatT octet_thr = std::sin(M_PI_4);
    IPoint current_point(pose.x, pose.y);
    FloatT cos_yaw = std::cos(pose.yaw);
    FloatT cos_yaw_sign = static_cast<int>(std::copysign(1.0, cos_yaw));
    FloatT sin_yaw = std::sin(pose.yaw);
    FloatT sin_yaw_sign = static_cast<int>(std::copysign(1.0, sin_yaw));
    int dx1 = std::abs(cos_yaw) > octet_thr? cos_yaw_sign : 0;
    int dy1 = dx1 == 0? sin_yaw_sign : 0;
    int dx2 = cos_yaw_sign;
    int dy2 = sin_yaw_sign;
    CellState current_state = check_cell_state(current_point, occ_thr, free_thr);
    FloatT range = INVALID_RANGE;
    FloatT last_distance = 0;
    while (current_state == CellState::FREE)
    {
        IPoint point1(current_point.x + dx1, current_point.y + dy1);
        IPoint point2(current_point.x + dx2, current_point.y + dy2);
        FloatT d1 = std::abs(cos_yaw * (pose.y - point1.y) - sin_yaw*(pose.x-point1.x));
        FloatT d2 = std::abs(cos_yaw * (pose.y - point2.y) - sin_yaw*(pose.x-point2.x));
        if (d1 < d2)
        {
            current_point = point1;
            last_distance = d1;
        }
        else
        {
            current_point = point2;
            last_distance = d2;
        }
        
        current_state = check_cell_state(current_point, occ_thr, free_thr);
    }
    if(current_state == CellState::OCCUPIED)
    {
        FloatT final_distance = std::sqrt(sqr(current_point.x-pose.x) + sqr(current_point.y-pose.y)); 
        range = std::sqrt(sqr(final_distance) - sqr(last_distance)) * m_resolution; 
    }

    return range;
}

template <typename FloatT>
void OccupancyGrid<FloatT>::update(FloatT delta_x, FloatT delta_y, FloatT delta_yaw)
{
    //TODO
}

template <typename FloatT>
void OccupancyGrid<FloatT>::update_cell_probability(const IPoint &point, CellState state)
{
    //TODO
}

template <typename FloatT>
bool OccupancyGrid<FloatT>::is_in_grid_bounds(const IPoint &point)
{
    if (point.x >= 0 && !(point.x < width())) return false;
    if (point.y >= 0 && !(point.y < height())) return false;
    return true;
}

template <typename FloatT>
Box<int> OccupancyGrid<FloatT>::get_map_box() const
{
    Box<int> map_box;
    for (int r = 0; r < m_map.rows(); ++r)
    {
        for (int c = 0; c < m_map.cols(); ++c)
        {
            FloatT value = m_map(r, c);
            if (value < 0)
            {
                continue;
            }
            map_box.min_x = c < map_box.min_x ? c : map_box.min_x;
            map_box.min_y = r < map_box.min_y ? r : map_box.min_y;
            map_box.max_x = c > map_box.max_x ? c : map_box.max_x;
            map_box.max_y = r > map_box.max_y ? r : map_box.max_y;
        }
    }

    return map_box;
}

}  // namespace slam
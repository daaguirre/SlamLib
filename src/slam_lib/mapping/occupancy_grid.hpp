
#include <assert.h>

#include <Eigen/Eigen>
#include <cmath>

#include "occupancy_grid.h"
#include "rclcpp/rclcpp.hpp"
#include "slam_lib/utils/math_helpers.h"

namespace slam
{

template <typename FloatT>
const typename OccupancyGrid<FloatT>::Point OccupancyGrid<FloatT>::INVALID_POSITION = {-1, -1};

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
typename OccupancyGrid<FloatT>::GridPose OccupancyGrid<FloatT>::to_map_frame(const Pose &pose2d)
{
    GridPose grid_pose;
    grid_pose.vector() = (m_transform * pose2d.vector()).template cast<int>();
    grid_pose.yaw = wrap_to_pi_range(pose2d.yaw + M_PI);
    return grid_pose;
}

template <typename FloatT>
typename OccupancyGrid<FloatT>::Pose OccupancyGrid<FloatT>::to_world_frame(const GridPose &pose)
{
    Pose world_pose;
    world_pose.vector() = m_transform.inverse() * pose.vector().template cast<FloatT>();
    world_pose.yaw = wrap_to_pi_range(pose.yaw - M_PI);
    return world_pose;
}

template <typename FloatT>
typename OccupancyGrid<FloatT>::Point OccupancyGrid<FloatT>::project_ray(
    const GridPose &pose,
    const FloatT range)
{
    FloatT dx = range * std::cos(pose.yaw);
    FloatT dy = range * std::sin(pose.yaw);
    int new_x = pose.x + std::ceil(dx / m_resolution);
    int new_y = pose.y + std::ceil(dy / m_resolution);
    return {new_x, new_y};
}

template <typename FloatT>
CellState OccupancyGrid<FloatT>::check_cell_state(const Point &point)
{
    if (point.x < width() && point.x > 0 && point.y < height() && point.y > 0)
    {
        FloatT occ_prob = m_map(point.y, point.x);
        if (occ_prob <= OCC_THR && occ_prob >= 0)
        {
            return CellState::OCCUPIED;
        }
        else if (occ_prob < 0)
        {
            return CellState::UNKNOWN;
        }

        return CellState::FREE;
    }

    return CellState::OUT_OF_BOUNDS;
}

template <typename FloatT>
FloatT OccupancyGrid<FloatT>::ray_tracing(const GridPose &pose)
{
    Point current_point(pose.x, pose.y);
    CellState current_state = check_cell_state(current_point);
    FloatT range = current_state == CellState::FREE? 0 : INVALID_RANGE;
    int counter = 1;
    while (current_state == CellState::FREE)
    {
        range = m_resolution * counter++;
        current_point = project_ray(pose, range);
        current_state = check_cell_state(current_point);
        if (current_state >= CellState::UNKNOWN)
        {
            range = INVALID_RANGE;
        }
    }
    return range;
}

template <typename FloatT>
void OccupancyGrid<FloatT>::update(FloatT delta_x, FloatT delta_y, FloatT delta_yaw)
{
    // Map temp_map(height(), width());
    // temp_map.setConstant(0.5);
    // // Convert position delta to cell delta
    // FloatT delta_x_grid = delta_x / m_resolution;
    // FloatT delta_y_grid = delta_y / m_resolution;
    // FloatT cos_dyaw = cos(delta_yaw);
    // FloatT sin_dyaw = sin(delta_yaw);
    // Map transformation_matrix(3, 3);
    // transformation_matrix << cos_dyaw, -sin_dyaw, delta_x_grid, sin_dyaw, cos_dyaw, delta_y_grid,
    //     0.0, 0.0, 1.0;
    // for (size_t x = 0; x < width(); ++x)
    // {
    //     for (size_t y = 0; y < height(); ++y)
    //     {
    //         Vector old_indices(3);
    //         // Translate (x, y) to origin
    //         old_indices << x - m_grid_center.x, y - m_grid_center.y, 1.0;
    //         // Transform according to robot movement
    //         Vector new_indices = transformation_matrix * old_indices;
    //         // Translate back to map indices
    //         new_indices(0) += m_grid_center.x;
    //         new_indices(1) += m_grid_center.y;
    //         if (is_in_grid_bounds(floor(new_indices(0)), floor(new_indices(1))))
    //         {
    //             temp_map(floor(new_indices(0)), floor(new_indices(1))) = m_map(x, y);
    //         }
    //     }
    // }
    // m_map = temp_map;
}

// void OccupancyGrid::update(const std::vector<Point2d<FloatT>> &laser_scan)
// {
//     // Create vector of free cells and reserve approximate amount of memory for max possible
//     distance
//     // std::vector<Point2d<int>> free_cells;
//     // free_cells.reserve(floor(m_grid_size));
//     // for (const Point2d<FloatT> &point : laser_scan)
//     // {
//     //     // Convert position of detection to cell indices
//     //     Point2d<int> grid_point{floor(point.x / m_cell_size) + m_grid_center.x,
//     //                             floor(point.y / m_cell_size) + m_grid_center.y};
//     //     update_cell_probability(grid_point, CellState::OCCUPIED);
//     //     // Run Bresenham algorithm to get all free cells
//     //     get_free_cells(grid_point, free_cells);
//     //     for (const Point2d<int> &free_cell : free_cells)
//     //     {
//     //         update_cell_probability(free_cell, CellState::FREE);
//     //     }
//     //     free_cells.clear();
//     // }
// }

template <typename FloatT>
void OccupancyGrid<FloatT>::update_cell_probability(const Point &point, CellState state)
{
    // Calculate new log odds and add to current log odds
    FloatT log_prob{0.0};
    switch (state)
    {
        case CellState::FREE:
            log_prob = log(m_p_free / (1.0 - m_p_free));
            break;
        case CellState::OCCUPIED:
            log_prob = log(m_p_occ / (1.0 - m_p_occ));
            break;
        default:
            log_prob = log(m_p_prior / (1.0 - m_p_prior));
            break;
    }
    FloatT current_log_prob = log(m_map(point.x, point.y) / (1.0 - m_map(point.x, point.y)));
    current_log_prob += log_prob;
    // Convert log odds to probability and update cell
    m_map(point.x, point.y) = 1.0 - 1.0 / (1 + exp(current_log_prob));
}

template <typename FloatT>
void OccupancyGrid<FloatT>::get_free_cells(const Point &detection, std::vector<Point> &free_cells)
{
    int x_start = m_grid_center.x;
    int y_start = m_grid_center.y;
    int dx = abs(detection.x - x_start);
    int dy = -abs(detection.y - y_start);
    int sx = x_start < detection.x ? 1 : -1;
    int sy = y_start < detection.y ? 1 : -1;
    int error = dx / 2;
    int e2;

    while ((x_start != detection.x) && (y_start != detection.y))
    {
        free_cells.push_back({x_start, y_start});
        e2 = 2 * error;
        if (e2 > dy)
        {
            error += dy;
            x_start += sx;
        }
        if (e2 < dx)
        {
            error += dx;
            y_start += sy;
        }
    }
}

template <typename FloatT>
bool OccupancyGrid<FloatT>::is_in_grid_bounds(const Point &point)
{
    if (point.x >= 0 && !(point.x < width())) return false;
    if (point.y >= 0 && !(point.y < height())) return false;
    return true;
}

}  // namespace slam
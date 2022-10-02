
#include <assert.h>

#include <cmath>

#include "occupancy_grid.h"
#include "rclcpp/rclcpp.hpp"

namespace slam
{

template <typename FloatT>
OccupancyGrid<FloatT>::OccupancyGrid(
    size_t width,
    size_t height,
    FloatT resolution,
    const std::string &name)
    : m_width{width}, m_height{height}, m_resolution(resolution), m_name(name)
{
    m_size = width * height;
    m_grid_center = {width / 2, height / 2};
    m_map.resize(height, width);
    m_map.setConstant(0.5);
}

template <typename FloatT>
nav_msgs::msg::OccupancyGrid OccupancyGrid<FloatT>::to_ros_msg()
{
    nav_msgs::msg::OccupancyGrid occ_grid_msg;
    occ_grid_msg.info.width = m_width;
    occ_grid_msg.info.height = m_height;
    occ_grid_msg.info.resolution = m_resolution;
    occ_grid_msg.info.origin.position.x = -m_grid_center.x * m_resolution;
    occ_grid_msg.info.origin.position.y = -m_grid_center.y * m_resolution;
    occ_grid_msg.header.frame_id = "map";
    occ_grid_msg.info.origin.position.x = 0.0;
    occ_grid_msg.info.origin.position.y = 0.0;
    occ_grid_msg.info.origin.position.z = 0.0;
    occ_grid_msg.info.origin.orientation.x = 0.0;
    occ_grid_msg.info.origin.orientation.y = 0.0;
    occ_grid_msg.info.origin.orientation.z = 0.0;
    occ_grid_msg.info.origin.orientation.w = 1.0;
    // occ_grid_msg.data.resize(m_map.size());

    for (size_t i = 0; i < m_size; i++)
    {
        FloatT &occ_prob = m_map.data()[i];
        if (occ_prob == 0.5)
        {
            occ_grid_msg.data.push_back(-1);
        }
        else
        {
            occ_grid_msg.data.push_back(occ_prob * 100);
        }
    }
    return occ_grid_msg;
}

template <typename FloatT>
void OccupancyGrid<FloatT>::update(FloatT delta_x, FloatT delta_y, FloatT delta_yaw)
{
    Map temp_map(m_height, m_width);
    temp_map.setConstant(0.5);
    // Convert position delta to cell delta
    FloatT delta_x_grid = delta_x / m_resolution;
    FloatT delta_y_grid = delta_y / m_resolution;
    FloatT cos_dyaw = cos(delta_yaw);
    FloatT sin_dyaw = sin(delta_yaw);
    Map transformation_matrix(3, 3);
    transformation_matrix << cos_dyaw, -sin_dyaw, delta_x_grid, sin_dyaw, cos_dyaw, delta_y_grid,
        0.0, 0.0, 1.0;
    for (size_t x = 0; x < m_width; ++x)
    {
        for (size_t y = 0; y < m_height; ++y)
        {
            Vector old_indices(3);
            // Translate (x, y) to origin
            old_indices << x - m_grid_center.x, y - m_grid_center.y, 1.0;
            // Transform according to robot movement
            Vector new_indices = transformation_matrix * old_indices;
            // Translate back to map indices
            new_indices(0) += m_grid_center.x;
            new_indices(1) += m_grid_center.y;
            if (is_in_grid_bounds(floor(new_indices(0)), floor(new_indices(1))))
            {
                temp_map(floor(new_indices(0)), floor(new_indices(1))) = m_map(x, y);
            }
        }
    }
    m_map = temp_map;
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
void OccupancyGrid<FloatT>::update_cell_probability(const Point2d<int> &point, CellState state)
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
void OccupancyGrid<FloatT>::get_free_cells(
    const Point2d<int> &detection,
    std::vector<Point2d<int>> &free_cells)
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
bool OccupancyGrid<FloatT>::is_in_grid_bounds(size_t x, size_t y)
{
    if (!(x < m_width)) return false;
    if (!(y < m_height)) return false;
    return true;
}

}  // namespace slam
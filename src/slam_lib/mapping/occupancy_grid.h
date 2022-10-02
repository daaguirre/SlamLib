/**
 * @file occupancy_grid.h
 * @author daaguirre
 * @brief
 * @date 2022-08-20
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __SRC_OCCUPANCY_GRID_H_
#define __SRC_OCCUPANCY_GRID_H_

#include <Eigen/Eigen>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
// #include "nav_msgs/msg/odometry.hpp"

namespace slam
{

template <typename T>
struct Point2d
{
    T x;
    T y;
};

enum class CellState : size_t
{
    FREE,
    OCCUPIED
};

// forward declaration
class MapReader;

class OccupancyGrid
{
public:
    using Map = Eigen::MatrixXd;

    OccupancyGrid(size_t width, size_t height, double resolution, const std::string &name);

    nav_msgs::msg::OccupancyGrid to_ros_msg();
    void update(double delta_x, double delta_y, double delta_yaw);

    size_t height() const
    {
        return m_height;
    }

    size_t width() const
    {
        return m_width;
    }

    size_t size() const
    {
        return m_size;
    }

    std::string name() const
    {
        return m_name;
    }
    
    const Map& map() const
    {
        return m_map;
    }
    // void update(const std::vector<Point2d<double>> &laser_scan);

    friend class MapReader;

private:
    void update_cell_probability(const Point2d<int> &point, CellState state);
    void get_free_cells(const Point2d<int> &detection, std::vector<Point2d<int>> &free_cells);
    bool is_in_grid_bounds(size_t x, size_t y);

    size_t m_width;
    size_t m_height;
    double m_resolution;
    std::string m_name;
    size_t m_size;
    Map m_map;
    Point2d<size_t> m_grid_center;
    const double m_p_free{0.4};
    const double m_p_occ{0.6};
    const double m_p_prior{0.5};
};

}  // namespace slam

#endif  // __SRC_OCCUPANCY_GRID_H_

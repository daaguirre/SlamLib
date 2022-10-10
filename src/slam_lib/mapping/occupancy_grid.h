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
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>

#include "slam_lib/types.h"

namespace slam
{

enum class CellState : size_t
{
    FREE,
    OCCUPIED,
    UNKNOWN,
    OUT_OF_BOUNDS
};

// forward declaration
template <typename FloatT>
class MapReader;

template <typename FloatT>
class OccupancyGrid
{
    static constexpr FloatT OCC_THR = 0.4;

public:
    using Map = Eigen::Matrix<FloatT, -1, -1, Eigen::ColMajor>;
    using Ptr = std::shared_ptr<OccupancyGrid<FloatT>>;
    using ConstPtr = std::shared_ptr<const OccupancyGrid<FloatT>>;
    using Point = PointXY<int>;
    using Pose = Pose2D<FloatT>;
    using Transform = Eigen::Transform<FloatT, 2, Eigen::Affine>;

    static const Point INVALID_POSITION;
    static const FloatT INVALID_RANGE;

    struct GridPose : public Point
    {
        GridPose() {}
        GridPose(const int x, const int y, const FloatT yaw) : Point(x, y), yaw(yaw) {}

        FloatT yaw;
    };

    OccupancyGrid(
        const int width,
        const int height,
        const FloatT resolution,
        const std::string& name);

    nav_msgs::msg::OccupancyGrid to_ros_msg();

    void update(FloatT delta_x, FloatT delta_y, FloatT delta_yaw);

    size_t height() const
    {
        return m_map.rows();
    }

    size_t width() const
    {
        return m_map.cols();
    }

    size_t size() const
    {
        return m_map.rows() * m_map.cols();
    }

    std::string name() const
    {
        return m_name;
    }

    const Map& map() const
    {
        return m_map;
    }

    FloatT resolution() const
    {
        return m_resolution;
    }

    GridPose to_map_frame(const Pose& pose2d);
    Pose to_world_frame(const GridPose& pose);

    /**
     * @brief projects a beam from pose a range distance and obtains
     * the corresponsing position in map.
     *
     * @param pose the pose of the beam which indicates the starting position and orientation
     * @param range the distance tavelled by the beam.
     * @return the point where the beam would stop.
     * If the ray is projected out of the map boundaries INVALID_POSITION is returned.
     */
    Point project_ray(const GridPose& pose, const FloatT range);

    /**
     * @brief obtains the distance from the input pose to first obstacle found.
     *
     * @param pose input pose
     * @return FloatT the range value to the first obstacle. If not obstacle is found then
     * INVALID_RANGE is returned
     */
    FloatT ray_tracing(const GridPose& pose);

    friend class MapReader<FloatT>;

private:
    using Vector = Eigen::Matrix<FloatT, -1, 1>;

    void update_cell_probability(const Point& point, CellState state);
    void get_free_cells(const Point& detection, std::vector<Point>& free_cells);
    bool is_in_grid_bounds(const Point& point);
    CellState check_cell_state(const Point& point);

    FloatT m_resolution;
    std::string m_name;
    Map m_map;
    Point m_grid_center;
    Transform m_transform;
    const FloatT m_p_free{0.6};
    const FloatT m_p_occ{0.4};
    const FloatT m_p_prior{0.5};
    const FloatT m_rt_step{0.1};  // ray tracing step
};

}  // namespace slam

#endif  // __SRC_OCCUPANCY_GRID_H_

/**
 * @file occupancy_grid_gpu.h
 * @author daaguirre
 * @brief
 * @date 2022-08-20
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __SLAM_LIB_MAPPING_OCCUPANCY_GRID_GPU_H_
#define __SLAM_LIB_MAPPING_OCCUPANCY_GRID_GPU_H_

#include "occupancy_grid.h"
#include "slam_lib/types.h"

namespace slam
{

template <typename FloatT>
class MapReader;

class OccupancyGridGPU
{
public:
    static constexpr float DEFAULT_OCC_THR = 0.15;
    static constexpr float DEFAULT_FREE_THR = 0.95;

    using Map = Eigen::Matrix<float, -1, -1, Eigen::ColMajor>;
    // using Ptr = std::shared_ptr<OccupancyGrid<float>>;
    // using ConstPtr = std::shared_ptr<const OccupancyGrid<float>>;
    using Transform = Eigen::Transform<float, 2, Eigen::Affine>;

    __host__ __device__
    OccupancyGridGPU(const int width, const int height, const float resolution);

    __host__ void setup_gpu_environment();
    __host__ void delete_gpu_environment();

    __host__ __device__ int width() const
    {
        return m_width;
    }

    __host__ __device__ int height() const
    {
        return m_height;
    }

    __host__ __device__ float resolution() const
    {
        return m_resolution;
    }

    // __device__ IPose<float> to_map_frame(const Pose<float>& pose2d) const;

    // __device__ Pose<float> to_world_frame(const IPose<float>& pose) const;

    /**
     * @brief projects a beam from pose a range distance and obtains
     * the corresponsing position in map.
     *
     * @param pose the pose of the beam which indicates the starting position and orientation
     * @param range the distance tavelled by the beam.
     * @return the point where the beam would stop.
     * If the ray is projected out of the map boundaries INVALID_POSITION is returned.
     */
    // __device__ IPoint project_ray(const IPose<float>& pose, const float range) const;

    /**
     * @brief obtains the distance from the input pose to first obstacle found.
     * The algorithm implemented here is based on brasenham's algorithm.
     * First a line equation is calculated
     * then the deltas are selected based on the octet where the yaw angle belongs
     * Deltas are applied and the closest position to the line is selected.
     * Finally range is calculated by using Pythagoras
     *
     * @param pose input pose
     * @param occ_thr occupancy threshold, any value smaller than this will consider the cell as
     * occupied
     * @param free_thr free threshold, any value bigger than this will consider the cell as free
     * @return float the range value to the first obstacle. If not obstacle is found then
     * INVALID_RANGE is returned
     */
    // __device__ float ray_tracing(
    //     const IPose<float>& pose,
    //     const float occ_thr = DEFAULT_OCC_THR,
    //     const float free_thr = DEFAULT_FREE_THR) const;

    __device__ CellState check_cell_state(
        const IPose<float>& pose,
        const float occ_thr = DEFAULT_OCC_THR,
        const float free_thr = DEFAULT_FREE_THR) const;

    __device__ CellState check_cell_state(
        const IPoint& point,
        const float occ_thr = DEFAULT_OCC_THR,
        const float free_thr = DEFAULT_FREE_THR) const;

private:
    const int m_width;
    const int m_height;
    const float m_resolution;

    float* m_d_map_ptr;
    Transform* m_d_transform_ptr;
};

}  // namespace slam

#endif  // __SLAM_LIB_MAPPING_OCCUPANCY_GRID_GPU_H_

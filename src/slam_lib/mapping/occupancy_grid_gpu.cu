/**
 * @file occupancy_grid_gpu.cu
 * @author daaguirre
 * @brief
 * @date 2022-08-20
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <cuda_tools/error.h>

#include "occupancy_grid_gpu.h"

namespace slam
{

__host__ OccupancyGridGPU::OccupancyGridGPU(
    const int width,
    const int height,
    const float resolution)
    : m_width(width), m_height(height), m_resolution(resolution)
{
}

// __device__ OccupancyGridGPU::OccupancyGridGPU(
//     const int width,
//     const int height,
//     const float resolution)
//     : m_width(width), m_height(height), m_resolution(resolution)
// {
// }

__host__ void OccupancyGridGPU::setup_gpu_environment()
{
    size_t map_bytes = sizeof(float) * m_width * m_height;
    CUDA_RT_CALL(cudaMalloc(&m_d_map_ptr, map_bytes));
    CUDA_RT_CALL(cudaMalloc(&m_d_transform_ptr, sizeof(Transform)));
}

__host__ void OccupancyGridGPU::delete_gpu_environment()
{
    CUDA_RT_CALL(cudaFree(m_d_map_ptr));
    CUDA_RT_CALL(cudaFree(m_d_transform_ptr));
}

__device__ CellState OccupancyGridGPU::check_cell_state(
    const IPose<float> &pose,
    const float occ_thr,
    const float free_thr) const
{
    const IPoint *point_ptr = dynamic_cast<const IPoint *>(&pose);
    return check_cell_state(*point_ptr);
}

CellState OccupancyGridGPU::check_cell_state(
    const IPoint &point,
    const float occ_thr,
    const float free_thr) const

{
    if (point.x < width() && point.x >= 0 && point.y < height() && point.y >= 0)
    {
        // column major order
        int idx = point.x + point.y * m_height;
        float occ_prob = m_d_map_ptr[idx];
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
}  // namespace slam
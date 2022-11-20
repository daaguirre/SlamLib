
#include "particle_filter_gpu.h"
#include "slam_lib/utils/math_helpers.h"

namespace slam
{
template <typename FloatT>
ParticleFilterGPU<FloatT>::ParticleFilterGPU(
    const size_t num_particles,
    typename OccupancyGrid<FloatT>::ConstPtr occ_grid)
{
    init_particles();
    size_t particles_bytes = sizeof(Particle) * num_particles;
    CUDA_RT_CALL(cudaMalloc(&this->d_particles, particles_bytes));
    CUDA_RT_CALL(cudaMemcpy(this->d_particles, this->m_particles.data(), particles_bytes, cudaMemcpyHostToDevice));
}

template <typename FloatT>
void ParticleFilterGPU<FloatT>::init_particles()
{
    m_num_particles_inv = 1.0 / static_cast<FloatT>(this->m_num_particles);
    this->m_particles.resize(this->m_num_particles);

    std::vector<IPoint> free_cells = this->m_occ_grid->get_free_cells();
    size_t n = free_cells.size();

    size_t count = 0;
    for(auto& particle : this->m_particles)
    {
        auto it = free_cells.begin();
        std::advance(it, rand() % n);
        *(static_cast<IPoint*>(&particle)) = *it;

        // yaw in [-pi, pi] range
        particle.yaw = wrap_to_pi_range((rand() / static_cast<FloatT>(RAND_MAX)) * 2 * M_PI);
        particle.weight = m_num_particles_inv;
        ++count;
    }
}

template <typename FloatT>
typename ParticleFilterGPU<FloatT>::Particle ParticleFilterGPU<FloatT>::update(
    const RobotReading<FloatT>& previous_odometry,
    const RobotReading<FloatT>& current_odometry)
{
    return Particle();
}

// template <typename FloatT>
// IPose<FloatT> ParticleFilterGPU<FloatT>::sample_motion_model(
//     const IPose<FloatT>& previous_map_pose,
//     const RobotReading<FloatT>& previous_reading,
//     const RobotReading<FloatT>& current_reading)
// {
// }

}  // namespace slam

#include "particle_filter_test.h"

#include <slam_lib/io/map_reader.h>
#include <slam_lib/mapping/occupancy_grid.h>
#include <slam_lib/particle_filter.h>
#include <slam_lib/ros/rviz_manager.h>
#include <slam_lib/utils/math_helpers.h>

#include <iostream>

TEST_F(ParticleFilterTest, test_particle_filter_initialization)
{
    using FloatT = float;
    using GridPose = slam::OccupancyGrid<FloatT>::GridPose;
    using Pose = slam::OccupancyGrid<FloatT>::Pose;

    std::string wean_map_path = (RESOURCES_DIR / "wean.dat").string();
    slam::MapReader<FloatT> map_reader;
    slam::OccupancyGrid<FloatT>::Ptr wean_map_ptr = map_reader.read_map(wean_map_path);

    const size_t num_particles = 1000;
    slam::ParticleFilter<FloatT> particle_filter(num_particles, wean_map_ptr);
    const auto& particles = particle_filter.get_particles();
    for(const auto& pose : particles)
    {
        EXPECT_EQ(slam::CellState::FREE, wean_map_ptr->check_cell_state(pose));
    }

    std::vector<Pose> poses(particles.size());
    std::transform(
        particles.begin(),
        particles.end(),
        poses.begin(),
        [wean_map_ptr](const auto& state) { return wean_map_ptr->to_world_frame(state); });

    slam::RVizManager<FloatT> rviz(m_node_ptr);
    rviz.publish_map(*wean_map_ptr);
    rviz.publish_pose_array(poses);
    rviz.wait_msgs();
}
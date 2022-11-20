set(lib_name SlamLib)

set(${lib_name}_sources
    slam_lib/io/map_reader.h
    slam_lib/io/map_reader.hpp
    slam_lib/io/map_reader.cpp
    slam_lib/io/odometry_reader.h
    slam_lib/io/odometry_reader.hpp
    slam_lib/io/odometry_reader.cpp
    slam_lib/mapping/occupancy_grid.h
    slam_lib/mapping/occupancy_grid.hpp
    slam_lib/mapping/occupancy_grid.cpp
    slam_lib/mapping/occupancy_grid_gpu.h
    slam_lib/mapping/occupancy_grid_gpu.cu
    slam_lib/odometry/odometry_models.h
    slam_lib/ros/rviz_manager.h
    slam_lib/ros/rviz_manager.hpp
    slam_lib/ros/rviz_manager.cpp
    slam_lib/types.h
    slam_lib/utils/math_helpers.h
    slam_lib/utils/math_helpers.hpp
    slam_lib/utils/math_helpers.cpp
    slam_lib/localization/particle_filter_base.h
    slam_lib/localization/particle_filter.h
    slam_lib/localization/particle_filter.hpp
    slam_lib/localization/particle_filter.cpp
    slam_lib/localization/particle_filter_gpu.h
    slam_lib/localization/particle_filter_gpu.cu
)

generate_cpp_sources(FILES ${${lib_name}_sources})

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_BINARY_DIR})
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_BINARY_DIR})
conan_cmake_configure(
    REQUIRES 
    eigen/3.4.0
    cuda_tools/0.1.0
    GENERATORS cmake_find_package)
conan_cmake_autodetect(settings)
conan_cmake_install(PATH_OR_REFERENCE .
                    BUILD missing
                    REMOTE conancenter
                    SETTINGS ${settings})

find_package(Eigen3 3.4.0 REQUIRED)
find_package(cuda_tools 0.1.0 REQUIRED)
find_package(OpenMP)
# find_package(OpenCV REQUIRED)
# find_package(PCL 1.12.1 REQUIRED COMPONENTS visualization)

# ROS Libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_definitions(${PCL_DEFINITIONS})
add_library(${lib_name} ${${lib_name}_sources})

target_include_directories(${lib_name} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
) 

ament_target_dependencies(${lib_name}
    rclcpp
    nav_msgs
    sensor_msgs
    tf2
    tf2_ros
)

target_link_libraries(${lib_name} 
    Eigen3::Eigen
    cuda_tools::cuda_tools
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    OpenMP::OpenMP_CXX
)

set_target_properties(${lib_name} PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

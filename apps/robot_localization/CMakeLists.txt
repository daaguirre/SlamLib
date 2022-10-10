# APPS
SET(app_name RobotLocalization)
SET(${app_name}_sources
        main.cpp
        robot_localization.h
        robot_localization.cpp
)

generate_cpp_sources(FILES ${${app_name}_sources})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(${app_name} ${${app_name}_sources})
ament_target_dependencies(${app_name}
    rclcpp
    nav_msgs
    geometry_msgs
    std_msgs
    tf2
    tf2_ros
)

target_link_libraries(${app_name} 
    SlamLib
)


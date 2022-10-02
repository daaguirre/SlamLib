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

add_executable(${app_name} ${${app_name}_sources})
ament_target_dependencies(${app_name}
    rclcpp
    nav_msgs
    # sensor_msgs
)

target_link_libraries(${app_name} 
    SlamLib
)


include(FetchContent)

FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.12.1
)
FetchContent_MakeAvailable(googletest)

add_definitions(-DRESOURCES_DIR_PATH="${CMAKE_CURRENT_LIST_DIR}/resources")

SET(app_name SlamLibTests)

SET(${app_name}_sources
    test_base.h
    test_base.cpp
    slam_lib/io/map_reader_test.h
    slam_lib/io/map_reader_test.cpp
    slam_lib/io/odometry_reader_test.h
    slam_lib/io/odometry_reader_test.cpp
    slam_lib/mapping/occupancy_grid_test.h
    slam_lib/mapping/occupancy_grid_test.cpp
    slam_lib/particle_filter_test.h
    slam_lib/particle_filter_test.cpp
)

generate_cpp_sources(FILES ${${app_name}_sources})

add_executable(${app_name} ${${app_name}_sources})

target_include_directories(${app_name} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
) 

target_link_libraries(${app_name}
  GTest::gtest_main
  SlamLib
)

add_test(project_name_gtest ${app_name})


include(GoogleTest)
gtest_discover_tests(${app_name})

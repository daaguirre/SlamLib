cmake_minimum_required(VERSION 3.5)
project(SlamLib VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
message("C++ compiler version ${CMAKE_CXX_COMPILER_VERSION}")
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "10.3.0")
    message(FATAL_ERROR "Insufficient gcc version")
  endif()
endif()


include(${CMAKE_SRC}/file.cmake)

enable_testing()
add_definitions(-DWORKSPACE_DIR="${CMAKE_CURRENT_LIST_DIR}")
add_subdirectory(src)
add_subdirectory(tests)
add_subdirectory(apps)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

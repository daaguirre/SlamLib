# Slam Lib

The slam lib is a C++ lbrary that implements different slam algorithms.
This library is still under development and below you can find the implemented algorithms so far:

* Localization
  * Particle filter: implemented a simple particle filter.

* Mapping
  * Occupancy grid: the mapping code is still to be implemented but it already implements some methods needed for helping in the localization task such as ray tracing and ray projection.

## Applications

The following applications are available:

### Robot localization

This app shows an example of robot localization given a map. For solving the localization problem the particle filter algorithm is used. The visualization part is implemented using ROS 2.

References:
<https://github.com/Mesywang/Particle-Filter-Localization>

Results:
![Alt text](apps/robot_localization/robot_localization.gif?raw=true "Title")

## Requirements

CMake
Eigen 3.3.7  
Boost 1.71

Compiler:
gcc 10.3.0

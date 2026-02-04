# Introduction
[![Industrial CI](https://github.com/vortexntnu/vortex-utils/actions/workflows/industrial-ci.yml/badge.svg)](https://github.com/vortexntnu/vortex-utils/actions/workflows/industrial-ci.yml)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/vortexntnu/vortex-utils/main.svg)](https://results.pre-commit.ci/latest/github/vortexntnu/vortex-utils/main)
[![codecov](https://codecov.io/github/vortexntnu/vortex-utils/graph/badge.svg?token=d6D7d5xNdf)](https://codecov.io/github/vortexntnu/vortex-utils)

This package contains common definitions and often-used utility functions in C++ and Python.
It consist of the packages `vortex_utils`, `vortex_utils_ros` and `vortex_utils_ros_tf`

# Usage

CMake:

```CMake
find_package(vortex_utils REQUIRED)

# For ROS utils
find_package(vortex_utils_ros REQUIRED)
find_package(vortex_utils_ros_tf REQUIRED)
```
For cpp packages link against the target:
```CMake
target_link_libraries(my_target vortex_utils)

# ROS libraries
target_link_libraries(my_target vortex_utils_ros)
# ROS tf2 transform library
target_link_libraries(my_target vortex_utils_ros_tf)

```
In Python, import your desired function/dataclass like for example:
```python
from vortex_utils.python_utils import ssa
```
To import from `vortex_utils_ros`
```python
from vortex_utils_ros.qos_profiles import sensor_data_profile, reliable_profile
```

In C++, include
```C++
#include <vortex/utils/math.hpp>
```
for mathematical functions,
```C++
#include <vortex/utils/types.hpp>
```
for common structs like 6DOF `PoseEuler`, `Pose` and `Twist`.
```C++
#include <vortex/utils/ros/qos_profiles.hpp>
```
for common QoS profile definitions from from `vortex_utils_ros`

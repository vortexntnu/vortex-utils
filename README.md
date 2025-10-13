# Introduction
[![Industrial CI](https://github.com/vortexntnu/vortex-utils/actions/workflows/industrial-ci.yml/badge.svg)](https://github.com/vortexntnu/vortex-utils/actions/workflows/industrial-ci.yml)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/vortexntnu/vortex-utils/main.svg)](https://results.pre-commit.ci/latest/github/vortexntnu/vortex-utils/main)
[![codecov](https://codecov.io/github/vortexntnu/vortex-utils/graph/badge.svg?token=d6D7d5xNdf)](https://codecov.io/github/vortexntnu/vortex-utils)

This package contains common definitions and often-used utility functions in C++ and Python.

# Usage

In Python, import your desired function/dataclass like for example:
```python
from vortex_utils.python_utils import ssa
```
```python
from vortex_utils.qos_profiles import sensor_data_profile, reliable_profile
```

In C++, include
```C++
#include <vortex/utils/math.hpp>
```
for mathematical functions,
```C++
#include <vortex/utils/qos_profiles.hpp>
```
for common QoS profile definitions, and
```C++
#include <vortex/utils/types.hpp>
```
for common structs like 6DOF `Eta` and `Nu`.

#include "vortex_utils/cpp_utils.hpp"

double vortex_utils::ssa(const double angle)
{
    double angle_ssa = fmod(angle + M_PI, 2 * M_PI) - M_PI;
    return angle_ssa;
}
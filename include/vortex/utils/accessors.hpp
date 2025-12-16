#ifndef VORTEX_UTILS__ACCESSORS_HPP_
#define VORTEX_UTILS__ACCESSORS_HPP_

#include "types.hpp"

namespace vortex::utils::types {

inline double x_of(const Eta& e) {
    return e.x;
}
inline double y_of(const Eta& e) {
    return e.y;
}
inline double z_of(const Eta& e) {
    return e.z;
}

inline double roll_of(const Eta& e) {
    return e.roll;
}
inline double pitch_of(const Eta& e) {
    return e.pitch;
}
inline double yaw_of(const Eta& e) {
    return e.yaw;
}

inline double x_of(const EtaQuat& e) {
    return e.x;
}
inline double y_of(const EtaQuat& e) {
    return e.y;
}
inline double z_of(const EtaQuat& e) {
    return e.z;
}

inline double qw_of(const EtaQuat& e) {
    return e.qw;
}
inline double qx_of(const EtaQuat& e) {
    return e.qx;
}
inline double qy_of(const EtaQuat& e) {
    return e.qy;
}
inline double qz_of(const EtaQuat& e) {
    return e.qz;
}

inline double x_of(const PoseQuatEigen& p) {
    return p.position.x();
}
inline double y_of(const PoseQuatEigen& p) {
    return p.position.y();
}
inline double z_of(const PoseQuatEigen& p) {
    return p.position.z();
}

inline double qw_of(const PoseQuatEigen& p) {
    return p.orientation.w();
}
inline double qx_of(const PoseQuatEigen& p) {
    return p.orientation.x();
}
inline double qy_of(const PoseQuatEigen& p) {
    return p.orientation.y();
}
inline double qz_of(const PoseQuatEigen& p) {
    return p.orientation.z();
}

}  // namespace vortex::utils::types

#endif  // VORTEX_UTILS__ACCESSORS_HPP_

#ifndef VORTEX_UTILS__CONCEPTS_HPP_
#define VORTEX_UTILS__CONCEPTS_HPP_

#include <concepts>
#include "accessors.hpp"

namespace vortex::utils::concepts {

template <typename T>
concept PositionLike = requires(const T& t) {
    { x_of(t) } -> std::convertible_to<double>;
    { y_of(t) } -> std::convertible_to<double>;
    { z_of(t) } -> std::convertible_to<double>;
};

template <typename T>
concept QuaternionLike = requires(const T& q) {
    { qw_of(q) } -> std::convertible_to<double>;
    { qx_of(q) } -> std::convertible_to<double>;
    { qy_of(q) } -> std::convertible_to<double>;
    { qz_of(q) } -> std::convertible_to<double>;
};

template <typename T>
concept EulerLike = requires(const T& t) {
    { roll_of(t) } -> std::convertible_to<double>;
    { pitch_of(t) } -> std::convertible_to<double>;
    { yaw_of(t) } -> std::convertible_to<double>;
};

template <typename T>
concept QuatPoseLike = PositionLike<T> && QuaternionLike<T> && (!EulerLike<T>);

template <typename T>
concept EulerPoseLike = PositionLike<T> && EulerLike<T> && (!QuaternionLike<T>);

template <typename T>
concept PoseLike = QuatPoseLike<T> || EulerPoseLike<T>;

}  // namespace vortex::utils::concepts

#endif  // VORTEX_UTILS__CONCEPTS_HPP_

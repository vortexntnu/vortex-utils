#ifndef VORTEX_UTILS__CONCEPTS_HPP_
#define VORTEX_UTILS__CONCEPTS_HPP_

#include <concepts>

#include "accessors.hpp"

namespace vortex::utils::concepts {

/**
 * @brief Concept for types that expose 3D positional components.
 *
 * A type satisfies PositionLike if it provides read access to
 * Cartesian coordinates via the accessors:
 *  - x_of(t)
 *  - y_of(t)
 *  - z_of(t)
 *
 * Each component must be convertible to double.
 *
 * @tparam T Type to be checked
 */
template <typename T>
concept PositionLike = requires(const T& t) {
    { x_of(t) } -> std::convertible_to<double>;
    { y_of(t) } -> std::convertible_to<double>;
    { z_of(t) } -> std::convertible_to<double>;
};

/**
 * @brief Concept for types that expose orientation as a quaternion.
 *
 * A type satisfies QuaternionLike if it provides read access to
 * quaternion components via the accessors:
 *  - qw_of(q)
 *  - qx_of(q)
 *  - qy_of(q)
 *  - qz_of(q)
 *
 * Each component must be convertible to double.
 *
 * @tparam T Type to be checked
 */
template <typename T>
concept QuaternionLike = requires(const T& q) {
    { qw_of(q) } -> std::convertible_to<double>;
    { qx_of(q) } -> std::convertible_to<double>;
    { qy_of(q) } -> std::convertible_to<double>;
    { qz_of(q) } -> std::convertible_to<double>;
};

/**
 * @brief Concept for types that expose orientation as Euler angles.
 *
 * A type satisfies EulerLike if it provides read access to
 * orientation expressed as roll, pitch, and yaw via the accessors:
 *  - roll_of(t)
 *  - pitch_of(t)
 *  - yaw_of(t)
 *
 * Each component must be convertible to double.
 *
 * @tparam T Type to be checked
 */
template <typename T>
concept EulerLike = requires(const T& t) {
    { roll_of(t) } -> std::convertible_to<double>;
    { pitch_of(t) } -> std::convertible_to<double>;
    { yaw_of(t) } -> std::convertible_to<double>;
};

/**
 * @brief Concept for pose-like types with quaternion orientation.
 *
 * A type satisfies QuatPoseLike if it:
 *  - Has positional components (PositionLike)
 *  - Has quaternion orientation (QuaternionLike)
 *  - Does NOT expose Euler orientation (not EulerLike)
 *
 * This prevents ambiguity for types that might provide both
 * Euler and quaternion accessors.
 *
 * @tparam T Type to be checked
 */
template <typename T>
concept QuatPoseLike = PositionLike<T> && QuaternionLike<T> && (!EulerLike<T>);

/**
 * @brief Concept for pose-like types with Euler angle orientation.
 *
 * A type satisfies EulerPoseLike if it:
 *  - Has positional components (PositionLike)
 *  - Has Euler orientation (EulerLike)
 *  - Does NOT expose quaternion orientation (not QuaternionLike)
 *
 * This prevents ambiguity for types that might provide both
 * Euler and quaternion accessors.
 *
 * @tparam T Type to be checked
 */
template <typename T>
concept EulerPoseLike = PositionLike<T> && EulerLike<T> && (!QuaternionLike<T>);

/**
 * @brief Concept for pose-like types with either Euler or quaternion
 * orientation.
 *
 * PoseLike is satisfied if the type represents a 3D pose with:
 *  - Position
 *  - Exactly one orientation representation:
 *    - quaternion (QuatPoseLike), or
 *    - Euler angles (EulerPoseLike)
 *
 * @tparam T Type to be checked
 */
template <typename T>
concept PoseLike = QuatPoseLike<T> || EulerPoseLike<T>;

}  // namespace vortex::utils::concepts

#endif  // VORTEX_UTILS__CONCEPTS_HPP_

#ifndef VORTEX_UTILS__CONCEPTS_HPP_
#define VORTEX_UTILS__CONCEPTS_HPP_

#include <concepts>

namespace vortex::utils::concepts {

/**
 * @brief Concept for types that expose 3D positional components.
 *
 * A type satisfies PositionLike if it contains public members:
 *
 *  - `x`
 *
 *  - `y`
 *
 *  - `z`
 *
 * Each component must be convertible to double.
 *
 * @tparam T Type to be checked
 */
template <typename T>
concept PositionLike = requires(const T& t) {
    { t.x } -> std::convertible_to<double>;
    { t.y } -> std::convertible_to<double>;
    { t.z } -> std::convertible_to<double>;
};

/**
 * @brief Concept for types that expose orientation as a quaternion.
 *
 * A type satisfies QuaternionLike if it contains public members:
 *
 *  - `qw`
 *
 *  - `qx`
 *
 *  - `qy`
 *
 *  - `qz`
 *
 * Each component must be convertible to double.
 *
 * @tparam T Type to be checked
 */
template <typename T>
concept QuaternionLike = requires(const T& t) {
    { t.qw } -> std::convertible_to<double>;
    { t.qx } -> std::convertible_to<double>;
    { t.qy } -> std::convertible_to<double>;
    { t.qz } -> std::convertible_to<double>;
};

/**
 * @brief Concept for types that expose orientation as Euler angles.
 *
 * A type satisfies EulerLike if it contains public members:
 *
 *  - `roll`
 *
 *  - `pitch`
 *
 *  - `yaw`
 *
 * Each component must be convertible to double.
 *
 * @tparam T Type to be checked
 */
template <typename T>
concept EulerLike = requires(const T& t) {
    { t.roll } -> std::convertible_to<double>;
    { t.pitch } -> std::convertible_to<double>;
    { t.yaw } -> std::convertible_to<double>;
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

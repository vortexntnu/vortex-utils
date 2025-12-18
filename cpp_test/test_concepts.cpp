#include <string>
#include <type_traits>

#include <eigen3/Eigen/Core>

#include "vortex/utils/concepts/accessors.hpp"
#include "vortex/utils/concepts/concepts.hpp"
#include "vortex/utils/concepts/views.hpp"

struct HasPositionAndEuler {
    double x, y, z;
    double roll, pitch, yaw;
};

struct HasPositionAndEulerExtra {
    double x, y, z;
    double roll, pitch, yaw;
    int extra;
};

struct HasPositionAndQuaternion {
    double x, y, z;
    double qw, qx, qy, qz;
};

struct PositionOnly {
    double x, y, z;
};

struct EulerOnly {
    double roll, pitch, yaw;
};

struct IncompleteEulerMembers {
    double x, y, z;
    double roll, pitch;  // missing yaw
};

struct IncompleteQuaternionMembers {
    double x, y, z;
    double qx, qy, qz;  // missing qw
};

struct WrongEulerMemberType {
    double x, y, z;
    std::string roll;  // wrong type
    double pitch, yaw;
};

struct HasEulerAndQuaternionMembers {
    double x, y, z;
    double roll, pitch, yaw;
    double qw, qx, qy, qz;
};

// ---------- Member Concepts ----------

static_assert(vortex::utils::concepts::HasPositionMembers<HasPositionAndEuler>);
static_assert(vortex::utils::concepts::HasEulerMembers<HasPositionAndEuler>);
static_assert(
    !vortex::utils::concepts::HasQuaternionMembers<HasPositionAndEuler>);

static_assert(
    vortex::utils::concepts::HasQuaternionMembers<HasPositionAndQuaternion>);
static_assert(
    !vortex::utils::concepts::HasEulerMembers<HasPositionAndQuaternion>);

static_assert(!vortex::utils::concepts::HasPositionMembers<EulerOnly>);
static_assert(
    !vortex::utils::concepts::HasEulerMembers<IncompleteEulerMembers>);
static_assert(!vortex::utils::concepts::HasQuaternionMembers<
              IncompleteQuaternionMembers>);

// ---------- PositionLike ----------

static_assert(vortex::utils::concepts::PositionLike<HasPositionAndEuler>);
static_assert(vortex::utils::concepts::PositionLike<HasPositionAndQuaternion>);
static_assert(vortex::utils::concepts::PositionLike<PositionOnly>);
static_assert(!vortex::utils::concepts::PositionLike<EulerOnly>);

// ---------- EulerLike ----------

static_assert(vortex::utils::concepts::EulerLike<HasPositionAndEuler>);
static_assert(vortex::utils::concepts::EulerLike<HasPositionAndEulerExtra>);
static_assert(!vortex::utils::concepts::EulerLike<HasPositionAndQuaternion>);
static_assert(!vortex::utils::concepts::EulerLike<IncompleteEulerMembers>);
static_assert(!vortex::utils::concepts::EulerLike<WrongEulerMemberType>);

// ---------- QuaternionLike ----------

static_assert(
    vortex::utils::concepts::QuaternionLike<HasPositionAndQuaternion>);
static_assert(!vortex::utils::concepts::QuaternionLike<HasPositionAndEuler>);
static_assert(
    !vortex::utils::concepts::QuaternionLike<IncompleteQuaternionMembers>);

// ---------- Pose Concepts ----------

static_assert(vortex::utils::concepts::EulerPoseLike<HasPositionAndEuler>);
static_assert(vortex::utils::concepts::EulerPoseLike<HasPositionAndEulerExtra>);
static_assert(
    !vortex::utils::concepts::EulerPoseLike<HasPositionAndQuaternion>);

static_assert(vortex::utils::concepts::QuatPoseLike<HasPositionAndQuaternion>);
static_assert(!vortex::utils::concepts::QuatPoseLike<HasPositionAndEuler>);

static_assert(!vortex::utils::concepts::PoseLike<HasEulerAndQuaternionMembers>,
              "Types exposing both Euler and Quaternion representations must "
              "be rejected");

static_assert(vortex::utils::concepts::PoseLike<HasPositionAndEuler>);
static_assert(vortex::utils::concepts::PoseLike<HasPositionAndQuaternion>);
static_assert(!vortex::utils::concepts::PoseLike<IncompleteEulerMembers>);
static_assert(!vortex::utils::concepts::PoseLike<WrongEulerMemberType>);

// ---------- Eigen Tests ----------

static_assert(
    !vortex::utils::concepts::EulerPoseLike<Eigen::Matrix<double, 6, 1>>,
    "Eigen::Vector6d should NOT satisfy EulerPoseLike without adapters");

static_assert(
    !vortex::utils::concepts::QuatPoseLike<Eigen::Matrix<double, 7, 1>>,
    "Eigen::Vector7d should NOT satisfy QuatPoseLike");

static_assert(!vortex::utils::concepts::PositionLike<Eigen::Vector3d>,
              "Eigen::Vector3d must NOT satisfy PositionLike");

// ---------- View Tests ----------

template <typename T>
concept AcceptsPosVector =
    requires(const T& t) { vortex::utils::views::pos_vector(t); };

template <typename T>
concept AcceptsOriVector =
    requires(const T& t) { vortex::utils::views::ori_vector(t); };

template <typename T>
concept AcceptsOriQuaternion =
    requires(const T& t) { vortex::utils::views::ori_quaternion(t); };

// ---------- pos_vector ----------

static_assert(AcceptsPosVector<HasPositionAndEuler>);
static_assert(AcceptsPosVector<HasPositionAndQuaternion>);
static_assert(AcceptsPosVector<PositionOnly>);
static_assert(AcceptsPosVector<IncompleteEulerMembers>);

// ---------- ori_vector ----------

static_assert(AcceptsOriVector<HasPositionAndEuler>);
static_assert(!AcceptsOriVector<HasPositionAndQuaternion>);
static_assert(!AcceptsOriVector<IncompleteEulerMembers>);
static_assert(!AcceptsOriVector<WrongEulerMemberType>);

// ---------- ori_quaternion ----------

static_assert(AcceptsOriQuaternion<HasPositionAndQuaternion>);
static_assert(!AcceptsOriQuaternion<HasPositionAndEuler>);
static_assert(!AcceptsOriQuaternion<PositionOnly>);

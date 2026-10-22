#ifndef VORTEX_UTILS_ROS_LANDMARK_NAMES_HPP
#define VORTEX_UTILS_ROS_LANDMARK_NAMES_HPP

#include <cstdint>
#include <optional>
#include <string_view>
#include <utility>

#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>

namespace vortex::utils::landmarks {

/// A LandmarkType constant and its name.
struct LandmarkTypeName {
    std::string_view name;
    std::uint16_t value;
};

/// A LandmarkSubtype constant, its name and the LandmarkType it belongs to.
struct LandmarkSubtypeName {
    std::string_view name;
    std::uint16_t value;
    std::uint16_t type;
};

using vortex_msgs::msg::LandmarkSubtype;
using vortex_msgs::msg::LandmarkType;

// Add new constants from LandmarkType.msg and LandmarkSubtype.msg here.
inline constexpr LandmarkTypeName kLandmarkTypes[] = {
    {"ARUCO_MARKER", LandmarkType::ARUCO_MARKER},
    {"ARUCO_BOARD", LandmarkType::ARUCO_BOARD},
    {"PIPELINE_START", LandmarkType::PIPELINE_START},
    {"PIPELINE_END", LandmarkType::PIPELINE_END},
    {"VALVE", LandmarkType::VALVE},
    {"GATE", LandmarkType::GATE},
    {"SLALOM_PIPE", LandmarkType::SLALOM_PIPE},
    {"TORPEDO_BOARD", LandmarkType::TORPEDO_BOARD},
    {"BIN", LandmarkType::BIN},
    {"PATH_MARKER", LandmarkType::PATH_MARKER},
    {"TABLE", LandmarkType::TABLE},
    {"OCTAGON", LandmarkType::OCTAGON},
    {"PINGER", LandmarkType::PINGER},
};

inline constexpr LandmarkSubtypeName kLandmarkSubtypes[] = {
    {"ARUCO_BOARD_CAMERA", LandmarkSubtype::ARUCO_BOARD_CAMERA,
     LandmarkType::ARUCO_BOARD},
    {"ARUCO_BOARD_SONAR", LandmarkSubtype::ARUCO_BOARD_SONAR,
     LandmarkType::ARUCO_BOARD},
    {"ARUCO_BOARD_DETECTION", LandmarkSubtype::ARUCO_BOARD_DETECTION,
     LandmarkType::ARUCO_BOARD},
    {"VALVE_VERTICAL", LandmarkSubtype::VALVE_VERTICAL, LandmarkType::VALVE},
    {"VALVE_HORIZONTAL", LandmarkSubtype::VALVE_HORIZONTAL,
     LandmarkType::VALVE},
    {"PIPELINE_START_CAMERA", LandmarkSubtype::PIPELINE_START_CAMERA,
     LandmarkType::PIPELINE_START},
    {"PIPELINE_START_SONAR", LandmarkSubtype::PIPELINE_START_SONAR,
     LandmarkType::PIPELINE_START},
    {"GATE_SEARCH_RESCUE", LandmarkSubtype::GATE_SEARCH_RESCUE,
     LandmarkType::GATE},
    {"GATE_SURVEY_REPAIR", LandmarkSubtype::GATE_SURVEY_REPAIR,
     LandmarkType::GATE},
    {"SLALOM_PIPE_WHITE", LandmarkSubtype::SLALOM_PIPE_WHITE,
     LandmarkType::SLALOM_PIPE},
    {"SLALOM_PIPE_RED", LandmarkSubtype::SLALOM_PIPE_RED,
     LandmarkType::SLALOM_PIPE},
    {"TORPEDO_BOARD_WHOLE", LandmarkSubtype::TORPEDO_BOARD_WHOLE,
     LandmarkType::TORPEDO_BOARD},
    {"TORPEDO_OPENING_LARGE_LEFT", LandmarkSubtype::TORPEDO_OPENING_LARGE_LEFT,
     LandmarkType::TORPEDO_BOARD},
    {"TORPEDO_OPENING_LARGE_RIGHT",
     LandmarkSubtype::TORPEDO_OPENING_LARGE_RIGHT, LandmarkType::TORPEDO_BOARD},
    {"TORPEDO_OPENING_SMALL_TOP", LandmarkSubtype::TORPEDO_OPENING_SMALL_TOP,
     LandmarkType::TORPEDO_BOARD},
    {"TORPEDO_OPENING_SMALL_BOTTOM",
     LandmarkSubtype::TORPEDO_OPENING_SMALL_BOTTOM,
     LandmarkType::TORPEDO_BOARD},
    {"BIN_SEARCH_RESCUE", LandmarkSubtype::BIN_SEARCH_RESCUE,
     LandmarkType::BIN},
    {"BIN_SURVEY_REPAIR", LandmarkSubtype::BIN_SURVEY_REPAIR,
     LandmarkType::BIN},
    {"BIN_UNCLASSIFIED", LandmarkSubtype::BIN_UNCLASSIFIED, LandmarkType::BIN},
    {"BIN_STRUCTURE", LandmarkSubtype::BIN_STRUCTURE, LandmarkType::BIN},
    {"PATH_MARKER_WHOLE", LandmarkSubtype::PATH_MARKER_WHOLE,
     LandmarkType::PATH_MARKER},
    {"TABLE_WHOLE", LandmarkSubtype::TABLE_WHOLE, LandmarkType::TABLE},
    {"TABLE_ITEM_NUTBOLT", LandmarkSubtype::TABLE_ITEM_NUTBOLT,
     LandmarkType::TABLE},
    {"TABLE_ITEM_ELECTRIC", LandmarkSubtype::TABLE_ITEM_ELECTRIC,
     LandmarkType::TABLE},
    {"TABLE_ITEM_PILL", LandmarkSubtype::TABLE_ITEM_PILL, LandmarkType::TABLE},
    {"TABLE_ITEM_BANDAID", LandmarkSubtype::TABLE_ITEM_BANDAID,
     LandmarkType::TABLE},
    {"TABLE_BASKET_SURVEY_REPAIR", LandmarkSubtype::TABLE_BASKET_SURVEY_REPAIR,
     LandmarkType::TABLE},
    {"TABLE_BASKET_SEARCH_RESCUE", LandmarkSubtype::TABLE_BASKET_SEARCH_RESCUE,
     LandmarkType::TABLE},
    {"OCTAGON_WHOLE", LandmarkSubtype::OCTAGON_WHOLE, LandmarkType::OCTAGON},
    {"OCTAGON_IMAGE_REPAIR", LandmarkSubtype::OCTAGON_IMAGE_REPAIR,
     LandmarkType::OCTAGON},
    {"OCTAGON_IMAGE_RESCUE", LandmarkSubtype::OCTAGON_IMAGE_RESCUE,
     LandmarkType::OCTAGON},
    {"OCTAGON_IMAGE_SEARCH", LandmarkSubtype::OCTAGON_IMAGE_SEARCH,
     LandmarkType::OCTAGON},
    {"OCTAGON_IMAGE_SURVEY", LandmarkSubtype::OCTAGON_IMAGE_SURVEY,
     LandmarkType::OCTAGON},
    {"PINGER_DEPLOY", LandmarkSubtype::PINGER_DEPLOY, LandmarkType::PINGER},
    {"PINGER_RESTORE", LandmarkSubtype::PINGER_RESTORE, LandmarkType::PINGER},
};

namespace detail {

template <typename Entry, std::size_t N>
constexpr std::optional<std::uint16_t> value_of(const Entry (&table)[N],
                                                std::string_view name) {
    for (const auto& entry : table) {
        if (entry.name == name) {
            return entry.value;
        }
    }
    return std::nullopt;
}

}  // namespace detail

/// Value of the LandmarkType constant with this name, e.g. "GATE".
constexpr std::optional<std::uint16_t> landmark_type_from_string(
    std::string_view name) {
    return detail::value_of(kLandmarkTypes, name);
}

/// Value of the LandmarkSubtype constant with this name.
constexpr std::optional<std::uint16_t> landmark_subtype_from_string(
    std::string_view name) {
    return detail::value_of(kLandmarkSubtypes, name);
}

}  // namespace vortex::utils::landmarks

#endif  // VORTEX_UTILS_ROS_LANDMARK_NAMES_HPP

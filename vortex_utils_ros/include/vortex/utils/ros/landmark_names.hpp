#ifndef VORTEX_UTILS_ROS_LANDMARK_NAMES_HPP
#define VORTEX_UTILS_ROS_LANDMARK_NAMES_HPP

#include <cstdint>
#include <optional>
#include <string_view>
#include <utility>

#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>

namespace vortex::utils::landmarks {

namespace detail {

using NameValue = std::pair<std::string_view, std::uint16_t>;
using vortex_msgs::msg::LandmarkSubtype;
using vortex_msgs::msg::LandmarkType;

// Add new constants from LandmarkType.msg and LandmarkSubtype.msg here.
inline constexpr NameValue kTypes[] = {
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

inline constexpr NameValue kSubtypes[] = {
    {"ARUCO_BOARD_CAMERA", LandmarkSubtype::ARUCO_BOARD_CAMERA},
    {"ARUCO_BOARD_SONAR", LandmarkSubtype::ARUCO_BOARD_SONAR},
    {"ARUCO_BOARD_DETECTION", LandmarkSubtype::ARUCO_BOARD_DETECTION},
    {"VALVE_VERTICAL", LandmarkSubtype::VALVE_VERTICAL},
    {"VALVE_HORIZONTAL", LandmarkSubtype::VALVE_HORIZONTAL},
    {"PIPELINE_START_CAMERA", LandmarkSubtype::PIPELINE_START_CAMERA},
    {"PIPELINE_START_SONAR", LandmarkSubtype::PIPELINE_START_SONAR},
    {"GATE_SEARCH_RESCUE", LandmarkSubtype::GATE_SEARCH_RESCUE},
    {"GATE_SURVEY_REPAIR", LandmarkSubtype::GATE_SURVEY_REPAIR},
    {"SLALOM_PIPE_WHITE", LandmarkSubtype::SLALOM_PIPE_WHITE},
    {"SLALOM_PIPE_RED", LandmarkSubtype::SLALOM_PIPE_RED},
    {"TORPEDO_BOARD_WHOLE", LandmarkSubtype::TORPEDO_BOARD_WHOLE},
    {"TORPEDO_OPENING_LARGE_LEFT", LandmarkSubtype::TORPEDO_OPENING_LARGE_LEFT},
    {"TORPEDO_OPENING_LARGE_RIGHT", LandmarkSubtype::TORPEDO_OPENING_LARGE_RIGHT},
    {"TORPEDO_OPENING_SMALL_TOP", LandmarkSubtype::TORPEDO_OPENING_SMALL_TOP},
    {"TORPEDO_OPENING_SMALL_BOTTOM", LandmarkSubtype::TORPEDO_OPENING_SMALL_BOTTOM},
    {"BIN_SEARCH_RESCUE", LandmarkSubtype::BIN_SEARCH_RESCUE},
    {"BIN_SURVEY_REPAIR", LandmarkSubtype::BIN_SURVEY_REPAIR},
    {"BIN_UNCLASSIFIED", LandmarkSubtype::BIN_UNCLASSIFIED},
    {"BIN_STRUCTURE", LandmarkSubtype::BIN_STRUCTURE},
    {"PATH_MARKER_WHOLE", LandmarkSubtype::PATH_MARKER_WHOLE},
    {"TABLE_WHOLE", LandmarkSubtype::TABLE_WHOLE},
    {"TABLE_ITEM_NUTBOLT", LandmarkSubtype::TABLE_ITEM_NUTBOLT},
    {"TABLE_ITEM_ELECTRIC", LandmarkSubtype::TABLE_ITEM_ELECTRIC},
    {"TABLE_ITEM_PILL", LandmarkSubtype::TABLE_ITEM_PILL},
    {"TABLE_ITEM_BANDAID", LandmarkSubtype::TABLE_ITEM_BANDAID},
    {"TABLE_BASKET_SURVEY_REPAIR", LandmarkSubtype::TABLE_BASKET_SURVEY_REPAIR},
    {"TABLE_BASKET_SEARCH_RESCUE", LandmarkSubtype::TABLE_BASKET_SEARCH_RESCUE},
    {"OCTAGON_WHOLE", LandmarkSubtype::OCTAGON_WHOLE},
    {"OCTAGON_IMAGE_REPAIR", LandmarkSubtype::OCTAGON_IMAGE_REPAIR},
    {"OCTAGON_IMAGE_RESCUE", LandmarkSubtype::OCTAGON_IMAGE_RESCUE},
    {"OCTAGON_IMAGE_SEARCH", LandmarkSubtype::OCTAGON_IMAGE_SEARCH},
    {"OCTAGON_IMAGE_SURVEY", LandmarkSubtype::OCTAGON_IMAGE_SURVEY},
    {"PINGER_DEPLOY", LandmarkSubtype::PINGER_DEPLOY},
    {"PINGER_RESTORE", LandmarkSubtype::PINGER_RESTORE},
};

template <std::size_t N>
constexpr std::optional<std::uint16_t> value_of(const NameValue (&table)[N],
                                                std::string_view name) {
    for (const auto& [key, value] : table) {
        if (key == name) {
            return value;
        }
    }
    return std::nullopt;
}

}  // namespace detail

/// Value of the LandmarkType constant with this name, e.g. "GATE".
constexpr std::optional<std::uint16_t> landmark_type_from_string(
    std::string_view name) {
    return detail::value_of(detail::kTypes, name);
}

/// Value of the LandmarkSubtype constant with this name.
constexpr std::optional<std::uint16_t> landmark_subtype_from_string(
    std::string_view name) {
    return detail::value_of(detail::kSubtypes, name);
}

}  // namespace vortex::utils::landmarks

#endif  // VORTEX_UTILS_ROS_LANDMARK_NAMES_HPP

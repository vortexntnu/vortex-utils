cmake_minimum_required(VERSION 3.20)

project(vortex_utils
  VERSION 0.1.0
  LANGUAGES CXX
)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(VORTEX_UTILS_BUILD_TESTS "Build vortex_utils tests" ON)

add_library(vortex_utils
  src/math.cpp
  src/waypoint_utils.cpp
)

add_library(vortex::utils ALIAS vortex_utils)

target_compile_features(vortex_utils PUBLIC cxx_std_20)

target_compile_options(vortex_utils PRIVATE
  -Wall
  -Wextra
  -Wpedantic
)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

target_link_libraries(vortex_utils
  PUBLIC
    Eigen3::Eigen
    yaml-cpp
)

target_include_directories(vortex_utils
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

set_target_properties(vortex_utils PROPERTIES
  EXPORT_NAME utils
)

install(
  TARGETS vortex_utils
  EXPORT vortex_utilsTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  EXPORT vortex_utilsTargets
  FILE vortex_utilsTargets.cmake
  NAMESPACE vortex::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vortex_utils
)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/vortex_utilsConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/vortex_utilsConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vortex_utils
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/vortex_utilsConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/vortex_utilsConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/vortex_utilsConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vortex_utils
)

if(VORTEX_UTILS_BUILD_TESTS)
  enable_testing()
  add_subdirectory(cpp_test)
endif()

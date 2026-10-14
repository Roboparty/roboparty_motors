# SPDX-License-Identifier: GPL-3.0
# Copyright (C) 2025-2026 Luo1imasi
cmake_minimum_required(VERSION 3.12)

file(READ "package.xml" PACKAGE_XML_CONTENT)
string(REGEX MATCH "<version>([0-9\\.]+)</version>" _ ${PACKAGE_XML_CONTENT})
project(roboparty_motors VERSION ${CMAKE_MATCH_1})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
if(NOT CMAKE_CROSSCOMPILING)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
endif()
set(CMAKE_CXX_COMPILER_LAUNCHER ccache)

set(THREADS_PREFER_PTHREAD_FLAG ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake QUIET)
find_package(Boost COMPONENTS system)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)

set(PUBLIC_DEPENDENCIES
    fmt::fmt spdlog::spdlog ${Boost_LIBRARIES} pthread Eigen3::Eigen)

add_subdirectory(src)

add_library(motors STATIC 
  src/motor_driver.cpp
)

target_include_directories(motors
  PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(motors PUBLIC ${PUBLIC_DEPENDENCIES} 
  dm_motors evo_motors lro_motors rp_motors xyn_motors
  motors_can motors_canfd
)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS motors dm_motors evo_motors lro_motors rp_motors xyn_motors motors_can motors_canfd
  EXPORT roboparty_motors-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(EXPORT roboparty_motors-targets
  FILE roboparty_motorsTargets.cmake
  NAMESPACE roboparty_motors::
  DESTINATION lib/cmake/roboparty_motors
)

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/roboparty_motorsConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/roboparty_motorsConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/roboparty_motorsConfig.cmake"
  INSTALL_DESTINATION lib/cmake/roboparty_motors
)

# Install auto-generated Config for non-ROS consumers (Deb mode)
install(FILES 
  "${CMAKE_CURRENT_BINARY_DIR}/roboparty_motorsConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/roboparty_motorsConfigVersion.cmake"
  DESTINATION lib/cmake/roboparty_motors
)

pybind11_add_module(motors_py
  src/pybind_module.cpp
)

target_include_directories(motors_py
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

target_link_libraries(motors_py PUBLIC motors)

set(PYTHON_INSTALL_DIR "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")

install(TARGETS motors_py
  LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}
  ARCHIVE DESTINATION ${PYTHON_INSTALL_DIR}
  RUNTIME DESTINATION ${PYTHON_INSTALL_DIR}
)

install(DIRECTORY scripts/motor_ota/
  DESTINATION ${PYTHON_INSTALL_DIR}/roboparty_motor_ota
  FILES_MATCHING PATTERN "*.py"
)

if(ament_cmake_FOUND)
    ament_export_libraries(motors dm_motors evo_motors lro_motors rp_motors xyn_motors motors_can motors_canfd)
    ament_export_include_directories(include)
    ament_package()
endif()

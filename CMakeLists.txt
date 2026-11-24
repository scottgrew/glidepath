cmake_minimum_required(VERSION 3.28)
# The prebuilt Foxglove SDK library targets macOS 15.5; linking for an older
# target makes the linker warn on every one of its objects.
set(CMAKE_OSX_DEPLOYMENT_TARGET 15.5 CACHE STRING "")
project(glidepath CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

enable_testing()

include(FetchContent)

set(BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_MODULE OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
  jsbsim
  GIT_REPOSITORY https://github.com/JSBSim-Team/jsbsim.git
  GIT_TAG v1.3.3
  GIT_SHALLOW TRUE
  SYSTEM
  EXCLUDE_FROM_ALL)

FetchContent_Declare(
  catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG v3.7.1
  GIT_SHALLOW TRUE
  SYSTEM)

FetchContent_MakeAvailable(jsbsim catch2)

# Eigen is header-only. SOURCE_SUBDIR names no CMakeLists, so FetchContent
# only fetches it: Eigen 3.4.0's own CMake would register ~1000 of its tests
# with ctest, and its old policies ignore every switch that turns them off.
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
  SOURCE_SUBDIR headers-only)
FetchContent_MakeAvailable(eigen)
add_library(Eigen3::Eigen INTERFACE IMPORTED)
target_include_directories(Eigen3::Eigen SYSTEM INTERFACE ${eigen_SOURCE_DIR})

FetchContent_Declare(
  mcap
  URL https://github.com/foxglove/mcap/archive/refs/tags/releases/cpp/v2.1.3.zip
  SOURCE_SUBDIR do-not-add)
FetchContent_MakeAvailable(mcap)
add_library(mcap_headers INTERFACE)
target_include_directories(mcap_headers SYSTEM INTERFACE
                            ${mcap_SOURCE_DIR}/cpp/mcap/include)

FetchContent_Declare(
  foxglove_sdk
  URL "https://github.com/foxglove/foxglove-sdk/releases/download/sdk/v0.27.0/foxglove-v0.27.0-cpp-x86_64-apple-darwin.zip"
  SOURCE_SUBDIR do-not-add)
FetchContent_MakeAvailable(foxglove_sdk)
list(APPEND CMAKE_PREFIX_PATH "${foxglove_sdk_SOURCE_DIR}")
find_package(foxglove-sdk CONFIG REQUIRED)
foxglove_sdk_add_cpp_library(foxglove_cpp TYPE STATIC REMOTE_ACCESS OFF)
get_target_property(_foxglove_includes foxglove_cpp INTERFACE_INCLUDE_DIRECTORIES)
if(_foxglove_includes)
  target_include_directories(foxglove_cpp SYSTEM INTERFACE ${_foxglove_includes})
endif()

find_library(ZSTD_LIBRARY NAMES zstd PATHS /usr/local/lib REQUIRED)
find_library(LZ4_LIBRARY NAMES lz4 PATHS /usr/local/lib REQUIRED)
target_link_libraries(foxglove_cpp PUBLIC
                      ${ZSTD_LIBRARY}
                      ${LZ4_LIBRARY}
                      "-framework IOKit"
                      "-framework CoreFoundation"
                      "-framework Security"
                      "-framework SystemConfiguration")

include(${catch2_SOURCE_DIR}/extras/Catch.cmake)

add_library(glidepath_warnings INTERFACE)
target_compile_options(
  glidepath_warnings
  INTERFACE -Wall -Wextra -Werror -Wconversion -Wsign-conversion -Wshadow)

add_library(glidepath_fsw STATIC fsw/atmosphere.cpp fsw/autopilot.cpp
                                            fsw/ekf.cpp fsw/estimator.cpp
                                            fsw/geodesy.cpp fsw/gnss.cpp
                                            fsw/engine_monitor.cpp
                                            fsw/ground.cpp
                                            fsw/guidance.cpp fsw/integrity.cpp
                                            fsw/landing.cpp fsw/takeoff.cpp)
target_include_directories(glidepath_fsw PUBLIC ${PROJECT_SOURCE_DIR})
target_link_libraries(glidepath_fsw PUBLIC Eigen3::Eigen
                                             PRIVATE glidepath_warnings)

add_library(glidepath_sim STATIC sim/gnss_errors.cpp sim/plant.cpp
                                          sim/sensors.cpp)
target_link_libraries(glidepath_sim PUBLIC glidepath_fsw PRIVATE libJSBSim
                                             glidepath_warnings)
target_compile_definitions(
  glidepath_sim PUBLIC GLIDEPATH_JSBSIM_ROOT="${jsbsim_SOURCE_DIR}")

add_library(glidepath_app STATIC app/flight.cpp app/telemetry.cpp)
target_link_libraries(glidepath_app PUBLIC glidepath_sim glidepath_fsw
                                          PRIVATE foxglove_cpp glidepath_warnings)

add_executable(glidepath app/main.cpp)
target_link_libraries(glidepath PRIVATE glidepath_app foxglove_cpp
                                      glidepath_warnings)

add_executable(glidepath_fsw_tests tests/fsw/test_autopilot.cpp
                                   tests/fsw/test_ekf.cpp
                                   tests/fsw/test_engine_monitor.cpp
                                   tests/fsw/test_gnss.cpp
                                   tests/fsw/test_estimator.cpp
                                   tests/fsw/test_ground.cpp
                                   tests/fsw/test_guidance.cpp
                                   tests/fsw/test_integrity.cpp
                                   tests/fsw/test_landing.cpp
                                   tests/fsw/test_takeoff.cpp)
target_link_libraries(
  glidepath_fsw_tests
  PRIVATE glidepath_fsw glidepath_warnings Catch2::Catch2WithMain)
catch_discover_tests(glidepath_fsw_tests)

add_executable(glidepath_tests tests/sim/test_gnss_errors.cpp
                               tests/sim/test_gnss_sensors.cpp
                               tests/sim/test_plant.cpp
                               tests/sim/test_sensors.cpp
                               tests/app/test_estimate.cpp
                               tests/sim/test_autopilot_loop.cpp
                               tests/app/test_scenarios.cpp
                               tests/app/test_telemetry.cpp
                               tests/app/test_approach.cpp
                               tests/app/test_autoland.cpp
                               tests/app/test_takeoff.cpp
                               tests/app/test_integrity_flight.cpp
                               tests/app/test_engine_out.cpp)
target_link_libraries(glidepath_tests PRIVATE glidepath_app foxglove_cpp
                                              mcap_headers ${ZSTD_LIBRARY}
                                              ${LZ4_LIBRARY} glidepath_warnings
                                              Catch2::Catch2WithMain)
catch_discover_tests(glidepath_tests)
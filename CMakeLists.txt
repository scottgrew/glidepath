cmake_minimum_required(VERSION 3.28)
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

include(${catch2_SOURCE_DIR}/extras/Catch.cmake)

add_library(glidepath_warnings INTERFACE)
target_compile_options(
  glidepath_warnings
  INTERFACE -Wall -Wextra -Werror -Wconversion -Wsign-conversion -Wshadow)

add_library(glidepath_core STATIC src/sim.cpp)
target_include_directories(glidepath_core PUBLIC src)
target_compile_definitions(
  glidepath_core PUBLIC GLIDEPATH_JSBSIM_ROOT="${jsbsim_SOURCE_DIR}")
target_link_libraries(glidepath_core PRIVATE libJSBSim glidepath_warnings)

add_executable(glidepath src/main.cpp)
target_link_libraries(glidepath PRIVATE glidepath_core glidepath_warnings)

add_executable(glidepath_tests tests/test_sim.cpp)
target_link_libraries(glidepath_tests PRIVATE glidepath_core glidepath_warnings
                                             Catch2::Catch2WithMain)
catch_discover_tests(glidepath_tests)
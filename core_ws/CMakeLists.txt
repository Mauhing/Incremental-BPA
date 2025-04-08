cmake_minimum_required(VERSION 3.10)
project(core_ibpa)

# ─────────────────────────────────────────────────────────────
# 🔧 Build configuration
# ─────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 🐞 Debug/Release toggle
option(BUILD_DEBUG "Build in Debug mode" ON)
if (BUILD_DEBUG)
  # Add debug symbols
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
  # Disable optimizations in Debug mode
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
  
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE)
  add_definitions(-D_DEBUG)
  # print CMAKE_CXX_FLAGS_DEBUG
  message(STATUS "CMAKE_CXX_FLAGS_DEBUG: ${CMAKE_CXX_FLAGS_DEBUG}")
else()
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()


# Add 'd' postfix for debug builds (affects all targets)
if (NOT DEFINED CMAKE_DEBUG_POSTFIX)
  set(CMAKE_DEBUG_POSTFIX "d")
endif()

# ─────────────────────────────────────────────────────────────
# 🧩 Third-party dependencies
# ─────────────────────────────────────────────────────────────

# yaml-cpp (in-source)
set(YAML_CPP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/yaml-cpp-0.8.0")
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
add_subdirectory(${YAML_CPP_DIR})  # defines yaml-cpp target

# Open3D
set(Open3D_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/open3d-devel-linux-x86_64-cxx11-abi-0.19.0/lib/cmake/Open3D")
find_package(Open3D REQUIRED)

# Threads
find_package(Threads REQUIRED)

# ─────────────────────────────────────────────────────────────
# 📂 Source files and headers
# ─────────────────────────────────────────────────────────────
# Enable all warnings and checks for unused code
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wconversion -Werror=conversion -Wunused -Wunused-function -Wunused-variable")

file(GLOB_RECURSE core_sources CONFIGURE_DEPENDS src/*.cpp)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${YAML_CPP_DIR}/include
)

# ─────────────────────────────────────────────────────────────
# 🧱 Library target
# ─────────────────────────────────────────────────────────────
add_library(core_ibpa_lib STATIC ${core_sources})

target_link_libraries(core_ibpa_lib PUBLIC
  yaml-cpp
  Open3D::Open3D
  Threads::Threads
)

target_include_directories(core_ibpa_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ─────────────────────────────────────────────────────────────
# 🚀 Optional standalone test executable
# ─────────────────────────────────────────────────────────────
if (CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
  add_executable(core_ibpa main.cpp)
  target_link_libraries(core_ibpa PRIVATE core_ibpa_lib)
endif()

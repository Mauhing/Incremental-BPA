cmake_minimum_required(VERSION 3.10)
project(core_ibpa)

# ─────────────────────────────────────────────────────────────
# 🔧 Project settings
# ─────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Debug or release
option(BUILD_DEBUG "Build in Debug mode" ON)
if(BUILD_DEBUG)
  set(CMAKE_BUILD_TYPE Debug)
  add_definitions(-D_DEBUG)
else()
  set(CMAKE_BUILD_TYPE Release)
endif()

# ─────────────────────────────────────────────────────────────
# 🧩 Third-party dependencies
# ─────────────────────────────────────────────────────────────

# yaml-cpp
set(YAML_CPP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/yaml-cpp-0.8.0")
add_subdirectory(${YAML_CPP_DIR})  # defines target yaml-cpp

# Open3D
set(Open3D_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/open3d-devel-linux-x86_64-cxx11-abi-0.18.0/lib/cmake/Open3D")
find_package(Open3D REQUIRED)

# pthreads
find_package(Threads REQUIRED)

# ─────────────────────────────────────────────────────────────
# 📚 Core library
# ─────────────────────────────────────────────────────────────

# Collect source files
file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS "src/*.cpp")

# Define the static library
add_library(core_ibpa_lib STATIC ${SRC_FILES})

# Include directories for consumers
target_include_directories(core_ibpa_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${YAML_CPP_DIR}/include
  ${Open3D_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(core_ibpa_lib PUBLIC
  yaml-cpp
  ${Open3D_LIBRARIES}
  Threads::Threads
)

# ─────────────────────────────────────────────────────────────
# 🧪 Optional test/main executable
# ─────────────────────────────────────────────────────────────

if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
  add_executable(core_ibpa main.cpp)
  target_link_libraries(core_ibpa PRIVATE core_ibpa_lib)
endif()

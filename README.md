# Real-Time Surface Reconstruction (Incremental BPA)

Author: Mauhing Yip

This repository provides an incremental Ball Pivoting Algorithm (BPA) pipeline for real-time and offline surface reconstruction from point clouds.

## Overview

The project includes two primary execution paths:

- Offline C++ executable with Open3D visualization in `core_ws`
- ROS wrapper for online processing from topics or bag files in `catkin_ws`

This implementation is based on:

> Digne, Julie. "An analysis and implementation of a parallel ball pivoting algorithm." Image Processing On Line 4 (2014): 149-168.

## Requirements

- CMake 3.10 or newer
- C++20-compatible compiler toolchain
- Open3D 0.18.0 (download and place in `core_ws/third_party`):
  https://github.com/isl-org/Open3D/releases#release-v0.18.0
- yaml-cpp 0.8.0 (already included in `core_ws/third_party`):
  https://github.com/jbeder/yaml-cpp/releases#release-0.8.0
- ROS (required only for `catkin_ws`)

## Quick Start

### Offline (recommended for first run)

1. Download sample TXT data from [Google Drive sample data](https://drive.google.com/drive/folders/1akYV00Lk5WZvFLY2Amjr0OE8SpZRXwcv?usp=sharing) into `01-data/01-input`.
2. Build:

   ```bash
   cd core_ws
   ./rebuild.sh
   cd ..
   ```

3. Run one of the following:

   - Point cloud with normals:

     ```bash
     ./core_ws/build/core_ibpa -c ./config_demo_robot_normal_points.yaml
     ```

   - Point cloud without normals (uses sensor pose):

     ```bash
     ./core_ws/build/core_ibpa -c ./config_demo_robot_pose_points.yaml
     ```

### Online (ROS)

1. Download sample ROS bag data from [Google Drive sample data](https://drive.google.com/drive/folders/1akYV00Lk5WZvFLY2Amjr0OE8SpZRXwcv?usp=sharing) into `01-data/01-input`.
2. Start ROS master:

   ```bash
   roscore
   ```

3. Play a bag file (example):

   ```bash
   rosbag play -r 1 /path/to/your.bag
   ```

4. Build and launch wrapper:

   ```bash
   cd catkin_ws
   ./rebuild.sh
   ```

The launch file `catkin_ws/src/ros_wrapper_ibpa/launch/ibpa.launch` passes `config_path` to the node. Update it to match your YAML configuration file.

## Repository Layout

- `core_ws/`: C++ library and offline executable (`core_ibpa`)
- `catkin_ws/`: ROS wrapper node and launch files
- `config*.yaml`: Dataset-specific configuration examples
- `01-data/`: Input/output data directory

## Configuration (YAML)

Minimal annotated example:

```yaml
# Surface Reconstruction Configuration
radius: 1.0                   # BPA ball radius; also sets octree depth
max_orphan_per_voxel: 1       # Max vertices kept per octree leaf after downsampling
reading_per_batch: 5          # Number of frames fused per batch

main_mesh_policy:
  enabled: true               # Reserved (parsed today, subject to change)
  execute_at_batch: 10        # Activation batch index if enabled is true

random:
  random_device: false        # true = non-deterministic seed
  seed: 42                    # used when random_device is false

seed_triangles_every_batch: false  # Search seed triangulation every batch
show_previous_vertices: false      # Visualize previously seen vertices

# Offline input (core_ws)
input_file: ./01-data/01-input/poses_and_points.txt
pcd_with_normal: false             # true if file contains per-point normals

# Output base path
output_file: ./01-data/02-output/Final_Mesh

# Online input (ROS wrapper)
down_sample_in_ros:
  enabled: true
  max_points: 1000

hole_length: 3.16                  # Boundary length threshold (visualization)
```

### Parameter Notes

- `radius`: Sets BPA radius and octree depth. Smaller values capture finer detail with higher computational cost.
- `max_orphan_per_voxel`: Leaf cap used during downsampling. Higher values retain more orphan vertices.
- `reading_per_batch`: Number of frames fused per iteration (latency vs. stability trade-off).
- `seed_triangles_every_batch`: Enables seed triangulation search every batch.
- `show_previous_vertices`: Toggles visualization of previously accumulated vertices.
- `down_sample_in_ros`: Enables subsampling for incoming ROS point clouds.
- `hole_length`: Viewer threshold for coloring long boundary edges (red).
- `main_mesh_policy`: If enabled, keeps only the largest edge-connected mesh after batch `N` (`execute_at_batch`). The same effect can be applied interactively by pressing `t` in the visualization window.

## Offline Input File Format

Input files are parsed in blocks delimited by `---`.

1. Without normals (`pcd_with_normal: false`), normals are estimated from pose:

```text
---
pose:
r11 r12 r13 tx
r21 r22 r23 ty
r31 r32 r33 tz
points:
x y z
x y z
...
---
```

2. With normals (`pcd_with_normal: true`), each point line contains `x y z nx ny nz`:

```text
---
points:
x y z nx ny nz
x y z nx ny nz
...
---
```

## Outputs

At program exit, the mesh is written as a PLY file via Open3D:

- Base path: `output_file` from YAML
- Output pattern: `<output_file>_YYYYMMDD_HHMMSS.ply`
- Example: `01-data/02-output/Final_Mesh_20250612_153045.ply`

## ROS Wrapper Details

- Subscribes to:
  - `/depth_registered/points` (`sensor_msgs/PointCloud2`)
  - `/rovio/odometry` (`nav_msgs/Odometry`)
- TF handling:
  - Attempts to read transform `imu_frame` -> `camera_frame`
  - Falls back to a hardcoded extrinsic transform if TF is unavailable. See line 68 in rosIbpaReconstructior.cpp
- Launch file:
  - `catkin_ws/src/ros_wrapper_ibpa/launch/ibpa.launch` (sets `config_path`)
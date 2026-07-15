Real-Time Surface Reconstruction (Incremental BPA)
=================================================

Author: Mauhing Yip

This project implements an incremental Ball Pivoting Algorithm (BPA) for real‑time or offline surface reconstruction from point clouds. It provides:
- An offline C++ executable with Open3D visualization (`core_ws`)
- A ROS wrapper for online processing from topics or bag files (`catkin_ws`)

We build on project: 
Digne, Julie. "An analysis and implementation of a parallel ball pivoting algorithm." Image Processing On Line 4 (2014): 149-168.

Build Requirements
------------------
- CMake ≥ 3.10, C++20 toolchain
- Open3D (Please download it and put it into `core_ws/third_party`) see https://github.com/isl-org/Open3D/releases#release-v0.18.0
- yaml-cpp (already included in `core_ws/third_party`) see https://github.com/jbeder/yaml-cpp/releases#release-0.8.0
- ROS (only for `catkin_ws` path)

Quick Start
-----------

Offline (recommended for first run)
- Download the sample files (txt) from [Google Drive sample data](https://drive.google.com/drive/folders/1akYV00Lk5WZvFLY2Amjr0OE8SpZRXwcv?usp=sharing) and put it into `/01-data/01-input`
- Build: `cd core_ws && ./rebuild.sh`
- Back to project dir `cd ..`
- For point cloud with normal 
  - Run: `./core_ws/build/core_ibpa -c ./config_demo_robot_nomral_points.yaml`
- For point cloud without normal, but sensor pose
  - Run: `./core_ws/build/core_ibpa -c ./config_demo_robot_pose_points.yaml`

Online (ROS)
- Download our sample ROS bag from [Google Drive sample data](https://drive.google.com/drive/folders/1akYV00Lk5WZvFLY2Amjr0OE8SpZRXwcv?usp=sharing) and put it into `/01-data/01-input`
- Start core services: `roscore`
- Play a bag (example): `rosbag play -r 1 /path/to/your.bag`
- Build + launch wrapper: `cd catkin_ws && ./rebuild.sh`
  - The launch file `ros_wrapper_ibpa/launch/ibpa.launch` passes `config_path` to the node. Edit it to point to your YAML.


Repository Layout
-----------------
- `core_ws/`: C++ library and offline executable (`core_ibpa`)
- `catkin_ws/`: ROS wrapper node and launch file
- `config*.yaml`: Example configurations (dataset‑specific)
- `01-data/`: Example input/output files and placeholders



Configuration (YAML)
--------------------

Minimal annotated example:
```yaml
# Surface Reconstruction Configuration
radius: 1.0                   # BPA ball radius; also sets octree depth
max_orphan_per_voxel: 1       # Max vertices kept per octree leaf after downsampling
reading_per_batch: 5          # Number of frames fused per batch

main_mesh_policy:
  enabled: true               # Reserved (parsed today, subject to change)
  execute_at_batch: 10        # Batch index to activate main mesh policy, only valid if enable is set to true.

random:
  random_device: false        # true = non-deterministic seed
  seed: 42                    # used when random_device=false

seed_triangles_every_batch: false  # Search seed triangulation each batch
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

Notes on parameters
- `radius`: Sets BPA radius and octree depth; smaller captures finer detail but increases computational cost.
- `max_orphan_per_voxel`: Leaf cap during downsampling; higher keeps more orphan vertices.
- `reading_per_batch`: Frames fused per iteration (trade latency vs. stability).
- `seed_triangles_every_batch`: Search seed triangulation each batch.
- `show_previous_vertices`: Toggles visualizing accumulated vertices for all previously detected vertices.
- `down_sample_in_ros`: Subsamples incoming ROS point clouds before fusion.
- `hole_length`: In the viewer, boundaries longer than this are colored as red.
- `main_mesh_policy`: It it is true, after the `N` batch, keep only the largest edge-connected mesh. If one wish the same effect after the batch `N`, one can press `t` in the visualzation windows.


Input File Format (offline)
---------------------------
Files are parsed in blocks delimited by `---`.

1) Without normals (`pcd_with_normal: false`) — normals are computed from pose:
```
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

2) With normals (`pcd_with_normal: true`) — each line carries x y z nx ny nz:
```
---
points:
x y z nx ny nz
x y z nx ny nz
...
---
```


Outputs
-------
- Mesh is saved as PLY using Open3D at program exit:
  - Base path: `output_file` from YAML
  - Actual file: `<output_file>_YYYYMMDD_HHMMSS.ply`
  - Example: `01-data/02-output/Final_Mesh_20250612_153045.ply`


ROS Wrapper Details
-------------------
- Subscribes: `/depth_registered/points` (sensor_msgs/PointCloud2), `/rovio/odometry` (nav_msgs/Odometry)
- TF: Tries to read transform `imu_frame` → `camera_frame`; falls back to a hardcoded extrinsic if TF is unavailable. (This mean that you may need to hardcode this yourself)
- Launch: `catkin_ws/src/ros_wrapper_ibpa/launch/ibpa.launch` sets `config_path`.


Examples
--------
Offline (Figaro example):
```
cd core_ws
./rebuild.sh
./build/core_ibpa -c ../config-Figaro.yaml
```

ROS with bag:
```
roscore
rosbag play -r 5 /path/to/your.bag
cd catkin_ws
./rebuild.sh   # builds, sources, and launches ibpa.launch
```

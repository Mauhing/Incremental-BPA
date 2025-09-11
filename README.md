Real-Time Surface Reconstruction (Incremental BPA)
=================================================

This project implements an incremental Ball Pivoting Algorithm (BPA) for real‑time or offline surface reconstruction from point clouds. It provides:
- An offline C++ executable with Open3D visualization (`core_ws`)
- A ROS wrapper for online processing from topics or bag files (`catkin_ws`)


Quick Start
-----------

Offline (recommended for first run)
- Build: `cd core_ws && ./rebuild.sh`
- Run: `./build/core_ibpa -c ../config-Figaro.yaml`

ROS (online)
- Start core services: `roscore`
- Play a bag (example): `rosbag play -r 5 /path/to/your.bag`
- Build + launch wrapper: `cd catkin_ws && ./rebuild.sh`
  - The launch file `ros_wrapper_ibpa/launch/ibpa.launch` passes `config_path` to the node. Edit it to point to your YAML.


Repository Layout
-----------------
- `core_ws/`: C++ library and offline executable (`core_ibpa`)
- `catkin_ws/`: ROS wrapper node and launch file
- `config*.yaml`: Example configurations (dataset‑specific)
- `01-data/`: Example input/output files and placeholders


Build Requirements
------------------
- CMake ≥ 3.10, C++20 toolchain
- Open3D (vendored in `core_ws/third_party` via CMake config path)
- yaml-cpp (vendored)
- ROS (only for `catkin_ws` path)


Configuration (YAML)
--------------------
Point the app to a YAML file via `-c <file.yaml>`. Examples: `config.yaml`, `config-Figaro.yaml`, `config-Nyhavna.yaml`.

Minimal annotated example:
```yaml
# Surface Reconstruction Configuration
radius: 1.0                   # BPA ball radius; also sets octree depth
max_orphan_per_voxel: 1       # Max vertices kept per octree leaf after downsampling
reading_per_batch: 5          # Number of frames fused per batch

main_mesh_policy:
  enabled: true               # Reserved (parsed today, subject to change)
  execute_at_batch: 10        # Batch index to activate main mesh policy

random:
  random_device: false        # true = non-deterministic seed
  seed: 42                    # used when random_device=false

seed_triangles_every_batch: false  # Reseed triangulation each batch
show_previous_vertices: false      # Visualize previously seen vertices

# Offline input (core_ws)
input_file: ./01-data/01-input/poses_and_points.txt
pcd_with_normal: false             # true if file contains per-point normals

# Output base path (timestamp is appended)
output_file: ./01-data/02-output/Final_Mesh

# Online input (ROS wrapper)
down_sample_in_ros:
  enabled: true
  max_points: 1000

hole_length: 3.16                  # Boundary length threshold (visualization)
```

Notes on parameters
- `radius`: Sets reconstruction scale and octree depth; smaller captures finer detail but increases cost.
- `max_orphan_per_voxel`: Leaf cap during downsampling; higher keeps more points (more detail/compute).
- `reading_per_batch`: Frames fused per iteration (trade latency vs. stability).
- `seed_triangles_every_batch`: Useful when the edge front disappears between batches.
- `show_previous_vertices`: Toggles visualizing accumulated vertices.
- `down_sample_in_ros`: Subsamples incoming ROS point clouds before fusion.
- `hole_length`: In the viewer, boundaries shorter than this are colored differently and may be considered “holes”.
- `main_mesh_policy`: Currently parsed and printed; behavior may evolve.


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
- TF: Tries to read transform `imu_frame` → `camera_frame`; falls back to a hardcoded extrinsic if TF is unavailable.
- Launch: `catkin_ws/src/ros_wrapper_ibpa/launch/ibpa.launch` sets `config_path`.


Troubleshooting
---------------
- No mesh produced: verify `input_file` exists and has the expected format; ensure `reading_per_batch` matches your data cadence.
- Sparse or noisy mesh: increase `max_orphan_per_voxel` or `radius`; verify normals (when using `pcd_with_normal: true`).
- Determinism for experiments: set `random_device: false` and fix `seed`.


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

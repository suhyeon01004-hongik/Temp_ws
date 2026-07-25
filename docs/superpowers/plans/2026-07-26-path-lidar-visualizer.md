# Path and LiDAR Visualizer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Align the IONIQ 5 RViz model with MORAI's rear-axle origin and add a vehicle-following path plus LiDAR view.

**Architecture:** Keep `morai_visualization` display-only. Correct the URDF debug geometry in `ioniq5_description`, publish vehicle-origin markers in `base_link`, and let RViz transform the existing `/lidar3D` point cloud through TF.

**Tech Stack:** ROS Noetic, catkin, xacro/URDF, C++14, RViz YAML, rostest, Python 3 unittest

## Global Constraints

- `base_link` is the midpoint of the rear-wheel axle.
- `base_footprint` is the ground projection of `base_link`.
- Existing path-only and LiDAR-only launch files remain usable.
- No LiDAR decoding, filtering, or accumulation is added.
- All user-facing documentation is Korean.

---

### Task 1: Rear-axle vehicle geometry

**Files:**
- Create: `src/ioniq5_description/test/test_vehicle_description.py`
- Modify: `src/ioniq5_description/config/vehicle_specs.yaml`
- Modify: `src/ioniq5_description/urdf/ioniq5_base.xacro`
- Modify: `src/ioniq5_description/urdf/molit_sensors.xacro`
- Modify: `src/ioniq5_description/CMakeLists.txt`
- Modify: `src/ioniq5_description/package.xml`

**Interfaces:**
- Consumes: vehicle dimensions and sensor mounts YAML
- Produces: `base_footprint -> base_link -> sensor_link` TF tree and corrected RobotModel

- [ ] **Step 1: Write the failing xacro behavior test**

Assert the expanded URDF has:

- body bounds `x=-0.790..3.845`
- `base_footprint -> base_link z=0.37`
- rear wheel centers at `x=0`
- front wheel centers at `x=3.0`
- unchanged GPS and LiDAR transforms

- [ ] **Step 2: Run the test and verify RED**

Run:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 src/ioniq5_description/test/test_vehicle_description.py
```

Expected: failure because the body is centered at `x=0`, the footprint transform is zero,
and wheel visuals do not exist.

- [ ] **Step 3: Implement the corrected xacro**

Add visualization-only height and axle/wheel dimensions to the YAML. Derive the body
center from wheelbase and overhang, raise `base_link` above `base_footprint`, add four
wheel visuals, and add a visible LiDAR model.

- [ ] **Step 4: Run the test and verify GREEN**

Run the same Python test and expect all assertions to pass.

### Task 2: Vehicle-origin marker behavior

**Files:**
- Create: `src/morai_visualization/test/path_visualizer.test`
- Create: `src/morai_visualization/test/test_path_visualizer.py`
- Modify: `src/morai_visualization/src/path_visualizer_node.cpp`
- Modify: `src/morai_visualization/config/path_visualizer.yaml`
- Modify: `src/morai_visualization/CMakeLists.txt`
- Modify: `src/morai_visualization/package.xml`

**Interfaces:**
- Consumes: `/localization/pose`
- Produces: `/visualization/path` markers with `vehicle_origin` and `vehicle_heading`
  in `base_link`

- [ ] **Step 1: Write the failing rostest**

Publish a localization pose and assert that the resulting marker array contains:

- `vehicle_origin` sphere in `base_link`
- `REAR AXLE` label in `base_link`
- `vehicle_heading` arrow in `base_link`
- no `current_position` namespace

- [ ] **Step 2: Run the test and verify RED**

Run:

```bash
catkin_make run_tests_morai_visualization
```

Expected: failure because the existing marker is named `current_position` and is in `map`.

- [ ] **Step 3: Implement the marker contract**

Allow markers to select their frame, use zero timestamp and frame locking for
vehicle-attached markers, and retain map-frame nearest-path calculations.

- [ ] **Step 4: Run the test and verify GREEN**

Run the focused package test and expect zero failures.

### Task 3: Combined RViz profile

**Files:**
- Create: `src/morai_visualization/test/test_visualization_assets.py`
- Create: `src/morai_visualization/launch/path_lidar.launch`
- Create: `src/morai_visualization/rviz/path_lidar.rviz`
- Modify: `src/morai_visualization/rviz/path.rviz`
- Modify: `src/morai_visualization/rviz/lidar.rviz`
- Modify: `src/morai_visualization/CMakeLists.txt`

**Interfaces:**
- Consumes: `/visualization/path`, `/global_path`, `/local_path`, `/lidar3D`,
  `robot_description`, and TF
- Produces: a combined `map`-fixed RViz view following `base_link`

- [ ] **Step 1: Write the failing profile test**

Parse the RViz and launch files and assert that the combined profile contains
MarkerArray, RobotModel, TF, and PointCloud2 `/lidar3D`, with Fixed Frame `map`
and Target Frame `base_link`.

- [ ] **Step 2: Run the test and verify RED**

Run:

```bash
python3 src/morai_visualization/test/test_visualization_assets.py
```

Expected: failure because `path_lidar.launch` and `path_lidar.rviz` do not exist.

- [ ] **Step 3: Add the combined launch/profile**

Reuse `path.launch` with RViz disabled, then start RViz with the combined profile.
Update the path profile namespace and keep the LiDAR-only profile independent of
localization.

- [ ] **Step 4: Run the test and verify GREEN**

Run the asset test and expect all assertions to pass.

### Task 4: Documentation and full verification

**Files:**
- Modify: `src/ioniq5_description/README.md`
- Modify: `src/morai_visualization/README.md`
- Modify: `src/morai_bringup/docs/CONFIGURATION_GUIDE_KO.md`

**Interfaces:**
- Consumes: completed launch, TF, marker, and geometry contracts
- Produces: Korean setup and troubleshooting guidance

- [ ] **Step 1: Document the coordinate convention and launch commands**

Explain rear-axle `base_link`, ground `base_footprint`, sensor positions, and the
separate path, LiDAR, and combined RViz launches.

- [ ] **Step 2: Build and run all tests**

Run:

```bash
catkin_make
catkin_make run_tests
catkin_test_results
```

Expected: successful build and zero failed tests.

- [ ] **Step 3: Inspect the final diff**

Run:

```bash
git diff --check
git status --short
```

Expected: no whitespace errors and only files in the approved scope.

### Task 5: Match the combined view to path-only and remove LiDAR flicker

**Files:**
- Modify: `src/morai_visualization/test/test_visualization_assets.py`
- Modify: `src/morai_visualization/rviz/path_lidar.rviz`
- Modify: `src/morai_visualization/rviz/lidar.rviz`
- Modify: `src/morai_visualization/README.md`

**Interfaces:**
- Consumes: the existing path-only RViz profile and `/lidar3D`
- Produces: a combined top-down profile that differs from path-only only by its
  PointCloud2 display

- [ ] **Step 1: Write the failing profile test**

Assert that `path_lidar.rviz` uses `rviz/TopDownOrtho` with the same Scale and
Target Frame as `path.rviz`, and that both PointCloud2 displays use
`Decay Time: 0.0`.

- [ ] **Step 2: Run the focused test and verify RED**

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 src/morai_visualization/test/test_visualization_assets.py
```

Expected: failure because the combined profile currently uses `rviz/Orbit` and
both LiDAR displays use `Decay Time: 0.15`.

- [ ] **Step 3: Apply the minimal profile change**

Replace the combined current/saved views with the exact `path.rviz`
`TopDownOrtho` view. Change only the PointCloud2 `Decay Time` in both LiDAR
profiles from `0.15` to `0.0`.

- [ ] **Step 4: Verify tests, install assets, and update Korean usage docs**

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin_make run_tests_morai_visualization
catkin_test_results build/test_results/morai_visualization
catkin_make install
```

Expected: all `morai_visualization` tests pass and the installed profiles contain
the top-down current view and zero decay time.

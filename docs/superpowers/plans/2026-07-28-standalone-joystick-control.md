# Standalone Joystick Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `vehicle_control` receive MORAI vehicle speed itself and map the CYVOX Home button to MORAI's initial-position reset without depending on another ROS package.

**Architecture:** A focused UDP receiver and packet parser publish a package-owned `VehicleStatus` message. The teleop node consumes that status for gear safety and publishes a reset request on the Home-button rising edge; a separate reset node sends the configured key to the MORAI X11 window.

**Tech Stack:** ROS1 Noetic, catkin, C++14, POSIX UDP sockets, `sensor_msgs/Joy`, package-owned ROS messages, `std_msgs/Empty`, `xdotool`, gtest, rostest.

## Global Constraints

- Work on `feature/standalone-joystick-control`.
- Do not depend on localization, path manager, UDP bridge, or their topics.
- Retain Cmd Control destination port `9093`.
- Receive MORAI Ego Vehicle Status on UDP port `9094` by default.
- Reject gear changes without a fresh speed or above `0.5 m/s`.
- Map Home/Guide button index `8` to one reset request per press.
- A reset failure must not stop joystick driving.

---

### Task 1: MORAI Vehicle Status Packet Parser

**Files:**
- Create: `src/vehicle_control/include/vehicle_control/morai_vehicle_status_packet.hpp`
- Create: `src/vehicle_control/src/morai_vehicle_status_packet.cpp`
- Create: `src/vehicle_control/test/test_morai_vehicle_status_packet.cpp`
- Modify: `src/vehicle_control/CMakeLists.txt`

**Interfaces:**
- Produces: `bool decodeMoraiVehicleStatus(const std::uint8_t*, std::size_t, MoraiVehicleStatus*, std::string*)`
- Produces: `MoraiVehicleStatus { std::int8_t control_mode; std::int8_t gear; float signed_speed_kph; }`

- [ ] **Step 1: Write failing parser tests**

Cover the current timestamped `#MoraiInfo$` layout, the legacy layout, invalid
header, truncated packet, unsupported data length, null output, and non-finite
speed. Construct byte arrays directly so the tests assert protocol offsets
rather than implementation helpers.

- [ ] **Step 2: Run the parser target and verify RED**

Run:

```bash
source /opt/ros/noetic/setup.bash
catkin_make tests
catkin_make run_tests_vehicle_control_gtest_test_morai_vehicle_status_packet
```

Expected: compilation fails because the parser header or implementation does
not exist.

- [ ] **Step 3: Implement the minimal parser**

Validate the 11-byte header, little-endian data length, required packet size,
and finite signed speed. Use speed offset `37` for timestamped layouts and
`29` for the legacy `132`-byte layout.

- [ ] **Step 4: Run parser and existing tests**

Run the parser target, then:

```bash
catkin_make run_tests_vehicle_control
catkin_test_results build/test_results/vehicle_control
```

Expected: zero errors and zero failures.

- [ ] **Step 5: Commit**

```bash
git add src/vehicle_control
git commit -m "feat: parse MORAI vehicle status packets"
```

### Task 2: Standalone MORAI Status UDP Node

**Files:**
- Create: `src/vehicle_control/msg/VehicleStatus.msg`
- Create: `src/vehicle_control/include/vehicle_control/udp_receiver.hpp`
- Create: `src/vehicle_control/src/udp_receiver.cpp`
- Create: `src/vehicle_control/src/morai_vehicle_status_udp_node.cpp`
- Create: `src/vehicle_control/test/test_udp_receiver.cpp`
- Modify: `src/vehicle_control/CMakeLists.txt`
- Modify: `src/vehicle_control/package.xml`

**Interfaces:**
- Consumes: `decodeMoraiVehicleStatus(...)`
- Produces: `/vehicle/status` as `vehicle_control/VehicleStatus`
- Produces: `UdpReceiver::receive(std::uint8_t*, std::size_t, std::size_t*, std::string*)`

- [ ] **Step 1: Write failing UDP loopback tests**

Test binding an ephemeral loopback port, receiving an exact datagram, rejecting
invalid addresses, rejecting a zero output buffer, and reporting an empty
nonblocking receive without blocking.

- [ ] **Step 2: Verify RED**

Run:

```bash
catkin_make tests
catkin_make run_tests_vehicle_control_gtest_test_udp_receiver
```

Expected: compilation fails because `UdpReceiver` does not exist.

- [ ] **Step 3: Implement receiver and node**

Use a nonblocking POSIX UDP socket. Parameters:

```yaml
listen_ip: 0.0.0.0
listen_port: 9094
status_topic: /vehicle/status
poll_rate: 200.0
```

Publish reception time, control mode, gear and signed speed in km/h.

- [ ] **Step 4: Verify GREEN and regression suite**

Run all `vehicle_control` tests and require zero failures.

- [ ] **Step 5: Commit**

```bash
git add src/vehicle_control
git commit -m "feat: receive MORAI vehicle status over UDP"
```

### Task 3: Remove Localization Dependency and Add Reset Edge

**Files:**
- Create: `src/vehicle_control/include/vehicle_control/button_edge.hpp`
- Create: `src/vehicle_control/src/button_edge.cpp`
- Create: `src/vehicle_control/test/test_button_edge.cpp`
- Modify: `src/vehicle_control/src/joystick_teleop_node.cpp`
- Modify: `src/vehicle_control/test/cyvox_mapping.test`
- Modify: `src/vehicle_control/test/test_cyvox_mapping.py`
- Modify: `src/vehicle_control/config/cyvox_mx.yaml`
- Modify: `src/vehicle_control/CMakeLists.txt`
- Modify: `src/vehicle_control/package.xml`

**Interfaces:**
- Consumes: `/vehicle/status` as `vehicle_control/VehicleStatus`
- Produces: `/vehicle/reset_request` as `std_msgs/Empty`
- Produces: `ButtonEdge::update(const std::vector<std::int32_t>&)` returning a rising-edge result

- [ ] **Step 1: Write failing button and rostests**

Verify Home index `8` emits once while held and again after release/repress.
Update the gear rostest to publish `VehicleStatus` instead of Odometry and
verify fresh low speed accepts, high speed rejects, and stale/missing status
rejects.

- [ ] **Step 2: Verify RED**

Run the button gtest and mapping rostest. Expected failures: missing
`ButtonEdge`, missing reset topic, and teleop still subscribing to Odometry.

- [ ] **Step 3: Implement minimal behavior**

Remove the Odometry subscriber and `nav_msgs` dependency. Convert absolute
`signed_speed_kph` to m/s, retain wall-time freshness checks, and publish an
empty reset request only on the configured button rising edge.

- [ ] **Step 4: Verify GREEN and regression suite**

Run all package tests and require zero failures.

- [ ] **Step 5: Commit**

```bash
git add src/vehicle_control
git commit -m "feat: make gear safety independent of localization"
```

### Task 4: MORAI Window Reset Node and User Configuration

**Files:**
- Create: `src/vehicle_control/include/vehicle_control/reset_command.hpp`
- Create: `src/vehicle_control/src/reset_command.cpp`
- Create: `src/vehicle_control/src/morai_sim_reset_node.cpp`
- Create: `src/vehicle_control/test/test_reset_command.cpp`
- Modify: `src/vehicle_control/config/cyvox_mx.yaml`
- Modify: `src/vehicle_control/launch/cyvox_morai.launch`
- Modify: `src/vehicle_control/CMakeLists.txt`
- Modify: `src/vehicle_control/package.xml`
- Modify: `src/vehicle_control/README.md`
- Modify: `README.md`

**Interfaces:**
- Consumes: `/vehicle/reset_request` as `std_msgs/Empty`
- Produces: `buildResetCommand(window_name, key)` with argv equivalent to:

```text
xdotool search --onlyvisible --name Simulator windowactivate --sync key --clearmodifiers i
```

- [ ] **Step 1: Write failing reset command tests**

Verify exact argv construction, rejection of an empty window name/key, and
process result handling for success, missing executable and nonzero exit.

- [ ] **Step 2: Verify RED**

Run:

```bash
catkin_make tests
catkin_make run_tests_vehicle_control_gtest_test_reset_command
```

Expected: compilation fails because the reset command API does not exist.

- [ ] **Step 3: Implement reset node**

Execute `xdotool` through argv without a shell. Parameters:

```yaml
reset_topic: /vehicle/reset_request
window_name: Simulator
reset_key: i
```

Log success or a precise error. Do not terminate the node on command failure.

- [ ] **Step 4: Update launch, config and Korean README**

Launch the status receiver and reset node by default. Document installation:

```bash
sudo apt install ros-noetic-joy xdotool
```

Document MORAI Cmd Control `9093`, Ego Vehicle Status Destination Port `9094`,
Home reset behavior, topics, parameters and troubleshooting.

- [ ] **Step 5: Run full verification**

```bash
source /opt/ros/noetic/setup.bash
catkin_make install
catkin_make run_tests_vehicle_control
catkin_test_results build/test_results/vehicle_control
roslaunch --nodes vehicle_control cyvox_morai.launch
```

Expected: build exit `0`, all tests pass, and launch lists joy, teleop, command
sender, status receiver and reset nodes.

- [ ] **Step 6: Commit**

```bash
git add README.md src/vehicle_control
git commit -m "feat: reset MORAI from the joystick Home button"
```

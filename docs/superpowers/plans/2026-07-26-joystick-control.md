# Joystick Vehicle Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a ROS1 Noetic `vehicle_control` package that maps the Joytron CYVOX MX to a neutral vehicle command and sends current MORAI Ego Ctrl Cmd UDP packets.

**Architecture:** `joy_node` publishes `/joy`; `joystick_teleop_node` maps axes to `VehicleCommand`; `morai_udp_sender_node` applies a stale-command safety policy and serializes the command to the current 59-byte MORAI UDP packet. Pure mapping, watchdog, packet, and socket behavior live in a tested C++14 core library.

**Tech Stack:** Ubuntu 20.04, ROS1 Noetic, catkin, C++14, `sensor_msgs/Joy`, generated ROS messages, POSIX UDP sockets, gtest.

## Global Constraints

- Work on the existing `feature/joystick-control` branch in `/home/suhyeon/catkin_ws`.
- Keep `morai_udp_bridge` sensor-only; all outgoing vehicle control belongs to `vehicle_control`.
- Use the detected CYVOX by-id path `/dev/input/by-id/usb-ShanWan_Xbox360_For_Windows_10F36D6-joystick`.
- Left-stick X is axis `0`, LT is axis `2`, and RT is axis `5`.
- Releasing RT publishes `accel=0` and `brake=0`; it must not apply automatic braking.
- A command older than `0.25 s` uses `accel=0`, `steering=0`, and `brake=0.5`.
- The first version fixes gear to MORAI Drive value `4`.
- The UDP packet uses the 2026 official 59-byte layout with a rear-steering float.
- All input mapping and UDP endpoint values must be editable in YAML.

---

### Task 1: Package and ROS command contract

**Files:**
- Create: `src/vehicle_control/package.xml`
- Create: `src/vehicle_control/CMakeLists.txt`
- Create: `src/vehicle_control/msg/VehicleCommand.msg`

**Interfaces:**
- Consumes: ROS Noetic `std_msgs/Header`.
- Produces: generated `vehicle_control::VehicleCommand` with `accel`, `brake`, `steering`, and `gear`.

- [ ] **Step 1: Create the message contract**

```text
std_msgs/Header header
float32 accel
float32 brake
float32 steering
uint8 GEAR_PARK=1
uint8 GEAR_REVERSE=2
uint8 GEAR_NEUTRAL=3
uint8 GEAR_DRIVE=4
uint8 GEAR_LOW=5
uint8 gear
```

- [ ] **Step 2: Create package metadata**

Declare `catkin`, `message_generation`, `message_runtime`, `roscpp`, `sensor_msgs`,
`std_msgs`, and runtime dependency `joy`. Use package format 2 and MIT license.

- [ ] **Step 3: Add message generation to CMake**

Use:

```cmake
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  sensor_msgs
  std_msgs
)
add_message_files(FILES VehicleCommand.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime roscpp sensor_msgs std_msgs)
```

- [ ] **Step 4: Build the generated message**

Run:

```bash
source /opt/ros/noetic/setup.bash
catkin_make install
```

Expected: exit `0` and generated `vehicle_control/VehicleCommand.h`.

- [ ] **Step 5: Commit**

```bash
git add src/vehicle_control/package.xml src/vehicle_control/CMakeLists.txt src/vehicle_control/msg/VehicleCommand.msg
git commit -m "feat: add vehicle control command contract"
```

### Task 2: Joystick mapping core

**Files:**
- Create: `src/vehicle_control/include/vehicle_control/control_command.hpp`
- Create: `src/vehicle_control/include/vehicle_control/joy_mapper.hpp`
- Create: `src/vehicle_control/src/joy_mapper.cpp`
- Create: `src/vehicle_control/test/test_joy_mapper.cpp`
- Modify: `src/vehicle_control/CMakeLists.txt`

**Interfaces:**
- Consumes: `std::vector<float>` axes and `JoyMappingConfig`.
- Produces: `bool JoyMapper::map(const std::vector<float>&, ControlCommand*, std::string*) const`.

- [ ] **Step 1: Write failing mapper tests**

Cover these hand-derived cases:

```cpp
TEST(JoyMapper, ReleasedTriggersProduceCoastingCommand) {
  const std::vector<float> axes{0.0F, 0.0F, -1.0F, 0.0F, 0.0F, -1.0F};
  ControlCommand output;
  ASSERT_TRUE(mapper.map(axes, &output, nullptr));
  EXPECT_FLOAT_EQ(0.0F, output.accel);
  EXPECT_FLOAT_EQ(0.0F, output.brake);
}

TEST(JoyMapper, FullyPressedTriggersProduceFullPedalCommands) {
  const std::vector<float> axes{0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 1.0F};
  ControlCommand output;
  ASSERT_TRUE(mapper.map(axes, &output, nullptr));
  EXPECT_FLOAT_EQ(1.0F, output.accel);
  EXPECT_FLOAT_EQ(1.0F, output.brake);
}

TEST(JoyMapper, SteeringDeadzoneIsRemovedAndRangeIsRescaled) {
  const std::vector<float> axes{0.525F, 0.0F, -1.0F, 0.0F, 0.0F, -1.0F};
  ControlCommand output;
  ASSERT_TRUE(mapper.map(axes, &output, nullptr));
  EXPECT_NEAR(0.5F, output.steering, 1.0e-6F);
}

TEST(JoyMapper, MissingConfiguredAxisRejectsInput) {
  ControlCommand output;
  std::string error;
  EXPECT_FALSE(mapper.map(std::vector<float>{0.0F}, &output, &error));
  EXPECT_FALSE(error.empty());
}
```

Also test steering inversion and non-finite input becoming a neutral value.

- [ ] **Step 2: Register and run the test to verify RED**

Add `catkin_add_gtest(test_joy_mapper test/test_joy_mapper.cpp)` and run:

```bash
source /opt/ros/noetic/setup.bash
catkin_make tests
catkin_make run_tests_vehicle_control_gtest_test_joy_mapper
```

Expected: compilation fails because `joy_mapper.hpp` and its implementation do not exist.

- [ ] **Step 3: Implement the minimal mapping library**

Define:

```cpp
struct ControlCommand {
  ControlCommand(float accel_value = 0.0F, float brake_value = 0.0F,
                 float steering_value = 0.0F,
                 std::uint8_t gear_value = 4U);
  float accel;
  float brake;
  float steering;
  std::uint8_t gear;
};

struct JoyMappingConfig {
  int steering_axis{0};
  int brake_axis{2};
  int accel_axis{5};
  bool steering_inverted{false};
  bool brake_inverted{false};
  bool accel_inverted{false};
  float steering_deadzone{0.05F};
};

class JoyMapper {
 public:
  explicit JoyMapper(JoyMappingConfig config);
  bool map(const std::vector<float>& axes, ControlCommand* output,
           std::string* error) const;
};
```

Use `(axis + 1) / 2` for non-inverted triggers. Clamp finite axes to
`[-1, 1]`; map non-finite steering to centered output `0` and non-finite
triggers to released output `0`. Rescale steering outside the deadzone with
`sign(x) * (abs(x)-deadzone)/(1-deadzone)`.

- [ ] **Step 4: Link and run GREEN**

Create `vehicle_control_core`, link the test to it, and run:

```bash
catkin_make run_tests_vehicle_control_gtest_test_joy_mapper
catkin_test_results --verbose
```

Expected: all mapper tests pass with zero failures.

- [ ] **Step 5: Commit**

```bash
git add src/vehicle_control/include src/vehicle_control/src/joy_mapper.cpp src/vehicle_control/test/test_joy_mapper.cpp src/vehicle_control/CMakeLists.txt
git commit -m "feat: map CYVOX axes to vehicle commands"
```

### Task 3: MORAI packet, timeout policy, and UDP transport

**Files:**
- Create: `src/vehicle_control/include/vehicle_control/command_watchdog.hpp`
- Create: `src/vehicle_control/include/vehicle_control/morai_ctrl_packet.hpp`
- Create: `src/vehicle_control/include/vehicle_control/udp_sender.hpp`
- Create: `src/vehicle_control/src/command_watchdog.cpp`
- Create: `src/vehicle_control/src/morai_ctrl_packet.cpp`
- Create: `src/vehicle_control/src/udp_sender.cpp`
- Create: `src/vehicle_control/test/test_command_watchdog.cpp`
- Create: `src/vehicle_control/test/test_morai_ctrl_packet.cpp`
- Create: `src/vehicle_control/test/test_udp_sender.cpp`
- Modify: `src/vehicle_control/CMakeLists.txt`

**Interfaces:**
- Consumes: `ControlCommand`, command age, destination IPv4 address and port.
- Produces: `ControlCommand CommandWatchdog::select(...)`, `MoraiCtrlPacket encodeMoraiCtrlPacket(...)`, and `UdpSender::send(...)`.

- [ ] **Step 1: Write failing watchdog tests**

```cpp
TEST(CommandWatchdog, FreshCommandPassesThrough) {
  const ControlCommand input{0.7F, 0.0F, -0.2F, 4U};
  const ControlCommand actual = watchdog.select(input, true, 0.1);
  EXPECT_FLOAT_EQ(0.7F, actual.accel);
  EXPECT_FLOAT_EQ(0.0F, actual.brake);
  EXPECT_FLOAT_EQ(-0.2F, actual.steering);
  EXPECT_EQ(4U, actual.gear);
}

TEST(CommandWatchdog, StaleCommandUsesConfiguredSafeBrake) {
  const ControlCommand actual =
      watchdog.select(ControlCommand{1.0F, 0.0F, 0.8F, 4U}, true, 0.3);
  EXPECT_FLOAT_EQ(0.0F, actual.accel);
  EXPECT_FLOAT_EQ(0.5F, actual.brake);
  EXPECT_FLOAT_EQ(0.0F, actual.steering);
  EXPECT_EQ(4U, actual.gear);
}
```

Also test the no-command state.

- [ ] **Step 2: Run watchdog tests to verify RED**

Expected: missing `command_watchdog.hpp` compilation failure.

- [ ] **Step 3: Implement watchdog and run GREEN**

Define:

```cpp
class CommandWatchdog {
 public:
  CommandWatchdog(double timeout_seconds, float safe_brake);
  ControlCommand select(const ControlCommand& latest, bool has_command,
                        double age_seconds) const;
};
```

Reject non-positive timeout in the constructor and clamp safe brake to `[0, 1]`.

- [ ] **Step 4: Write failing packet tests**

Assert the literal layout:

```text
0..13   "#MoraiCtrlCmd$"
14..17  little-endian int32 27
18..29  twelve zero auxiliary bytes
30      CtrlMode 2
31      gear 4
32      longCmdType 1
33..36  velocity float 0
37..40  acceleration float 0
41..44  accel float
45..48  brake float
49..52  front steering float
53..56  rear steering float 0
57..58  CR LF
```

Use literal expected bytes for `accel=0.5`, `brake=0.25`, and
`steering=-1.0`, and verify output size `59`.

- [ ] **Step 5: Run packet tests to verify RED**

Expected: missing packet encoder compilation failure.

- [ ] **Step 6: Implement explicit little-endian packet encoding and run GREEN**

Define:

```cpp
using MoraiCtrlPacket = std::array<std::uint8_t, 59U>;
MoraiCtrlPacket encodeMoraiCtrlPacket(const ControlCommand& command);
```

Use `memcpy` only to obtain IEEE-754 float bits, then write the four bytes
explicitly in little-endian order. Clamp command ranges before encoding.

- [ ] **Step 7: Write a failing real UDP loopback test**

Bind a test receiver to `127.0.0.1` on an ephemeral port, send the literal
three-byte payload `{0x10, 0x20, 0x30}`, receive it with a one-second socket
timeout, and compare the exact bytes.

- [ ] **Step 8: Implement UDP sender and run GREEN**

Define:

```cpp
class UdpSender {
 public:
  UdpSender(const std::string& destination_ip, std::uint16_t destination_port);
  ~UdpSender();
  void send(const std::uint8_t* data, std::size_t size) const;
};
```

Validate IPv4 with `inet_pton`, create one `AF_INET/SOCK_DGRAM` socket, and
require `sendto` to transmit the full datagram.

- [ ] **Step 9: Run all core tests**

```bash
catkin_make tests
catkin_make run_tests_vehicle_control
catkin_test_results --verbose
```

Expected: mapper, watchdog, packet, and UDP tests all pass.

- [ ] **Step 10: Commit**

```bash
git add src/vehicle_control/include src/vehicle_control/src src/vehicle_control/test src/vehicle_control/CMakeLists.txt
git commit -m "feat: add safe MORAI UDP command transport"
```

### Task 4: ROS nodes, configuration, launch, and documentation

**Files:**
- Create: `src/vehicle_control/src/joystick_teleop_node.cpp`
- Create: `src/vehicle_control/src/morai_udp_sender_node.cpp`
- Create: `src/vehicle_control/config/cyvox_mx.yaml`
- Create: `src/vehicle_control/launch/cyvox_morai.launch`
- Create: `src/vehicle_control/README.md`
- Modify: `src/vehicle_control/CMakeLists.txt`
- Modify: `README.md`

**Interfaces:**
- Consumes: `/joy` (`sensor_msgs/Joy`) and private YAML parameters.
- Produces: `/vehicle/manual_command` (`vehicle_control/VehicleCommand`) and UDP datagrams to `127.0.0.1:9095`.

- [ ] **Step 1: Implement `joystick_teleop_node` as thin ROS glue**

Load `steering_axis`, `brake_axis`, `accel_axis`, inversion flags, and
`steering_deadzone` from private parameters. Subscribe with queue size `1`,
call `JoyMapper`, throttle invalid-axis log output, and publish:

```cpp
message.header = joy.header;
message.accel = command.accel;
message.brake = command.brake;
message.steering = command.steering;
message.gear = vehicle_control::VehicleCommand::GEAR_DRIVE;
```

- [ ] **Step 2: Implement `morai_udp_sender_node` as thin ROS glue**

Load destination, rate, timeout, and safe brake. Subscribe with queue size
`1`, save the latest command and `ros::WallTime::now()`, and use a
`ros::WallTimer` at `50 Hz` to select the watchdog command, encode it, and
send it.

- [ ] **Step 3: Add executables and install rules**

Link both nodes to `vehicle_control_core` and `${catkin_LIBRARIES}`. Add
message target dependencies. Install the core library, nodes, public
headers, `config`, and `launch`.

- [ ] **Step 4: Add the CYVOX YAML**

Use:

```yaml
joy_node:
  dev: /dev/input/by-id/usb-ShanWan_Xbox360_For_Windows_10F36D6-joystick
  deadzone: 0.05
  autorepeat_rate: 20.0
  coalesce_interval: 0.001
  default_trig_val: false

joystick_teleop_node:
  joy_topic: /joy
  command_topic: /vehicle/manual_command
  steering_axis: 0
  brake_axis: 2
  accel_axis: 5
  steering_inverted: false
  brake_inverted: false
  accel_inverted: false
  steering_deadzone: 0.05

morai_udp_sender_node:
  command_topic: /vehicle/manual_command
  destination_ip: 127.0.0.1
  destination_port: 9095
  send_rate: 50.0
  command_timeout: 0.25
  safe_brake: 0.5
```

- [ ] **Step 5: Add one launch file**

Load the YAML inside a `vehicle_control` namespace, run upstream
`joy_node`, remap its `joy` output to `/joy`, then run both custom nodes.
Expose `config_file` as a launch argument.

- [ ] **Step 6: Document setup and operation in Korean**

Document:

```bash
sudo apt install ros-noetic-joy
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/install/setup.bash
roslaunch vehicle_control cyvox_morai.launch
rostopic echo /joy
rostopic echo /vehicle/manual_command
```

Explain MORAI `AV-ExternalCtrl`, Cmd Control IP/port matching, parameter
meanings, coasting behavior, disconnect braking, and steering inversion.
Update the root architecture, package table, topics, settings table, and
current limitations.

- [ ] **Step 7: Build and run all automated verification**

Install the upstream joystick runtime if it is missing:

```bash
source /opt/ros/noetic/setup.bash
if ! rospack find joy >/dev/null 2>&1; then
  sudo apt-get install -y ros-noetic-joy
fi
```

Then run:

```bash
source /opt/ros/noetic/setup.bash
catkin_make install
catkin_make tests
catkin_make run_tests_vehicle_control
catkin_test_results --verbose
```

Expected: build exits `0` and all `vehicle_control` tests pass.

- [ ] **Step 8: Perform read-only hardware smoke checks**

With the joystick connected:

```bash
test -r /dev/input/by-id/usb-ShanWan_Xbox360_For_Windows_10F36D6-joystick
source install/setup.bash
roslaunch vehicle_control cyvox_morai.launch
rostopic echo -n 1 /joy
rostopic echo -n 1 /vehicle/manual_command
```

Do not leave ROS nodes running after the checks. A full vehicle-motion check
requires the user to match MORAI Cmd Control port `9095` and select
`AV-ExternalCtrl`.

- [ ] **Step 9: Commit**

```bash
git add src/vehicle_control README.md
git commit -m "feat: control MORAI with CYVOX joystick"
```

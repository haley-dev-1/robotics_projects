# Running ROS2 Project 2a

## 1. Starting Application Order
The general order to begin your work is to open three terminals. In each
terminal, and in this order, run the following:

### 1.1 Start Gazebo on ros domain 67 to avoid cross talk with robot :
```bash
ROS_DOMAIN_ID=62  \
ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py \
  world_path:=/acct/halind/Documents/csce274/proj2a/worlds/lab.world \
  spawn_dock:=false \
  x:=0 y:=0

```
  - If others are working at the same time in the lab, you may all need to
    agree to different numbers and replace 67 with that number.

### 1.2 Start the script you are modifying :
```bash
colcon build --symlink-install
source install/setup.sh
ROS_DOMAIN_ID=62  \
ros2 run boustrophedon main
```
---

## 2. Simulate button-presses for the robot:
  This is how you send message to the robot. When we move to the actual bot, we
  will literally press the buttons.
```bash
ROS_DOMAIN_ID=62  \
ros2 topic pub --once  \
/interface_buttons  \
irobot_create_msgs/msg/InterfaceButtons "{  \
  button_1: {is_pressed: false},  \
  button_power: {is_pressed: true},  \
  button_2: {is_pressed: true}  \
}"
```

---

## 3. Expected responses before editing main.py:
[INFO] [1762973115.441728357] [main]: ROS_DOMAIN_ID: 67
[INFO] [1762973115.441961359] [main]: ROS_NAMESPACE:
[INFO] [1762973115.442155923] [main]: RMW_IMPLEMENTATION: rmw_fastrtps_cpp
[INFO] [1762973115.609663122] [odom_node]: OdomNode ready (listening to /odom).
[INFO] [1762973115.621498490] [ir_node]: IrNode ready (using native sensor order).
[INFO] [1762973115.629741297] [Controller]: Pulse started, calling pulse() every 1.0s.
[INFO] [1762973115.648768444] [button_node]: ButtonNode ready.
[INFO] [1762973115.662903041] [hazard_node]: HazardNode ready.
[INFO] [1762973115.667831534] [ir_node]: IR intensity messages are coming in!
[INFO] [1762973116.633402923] [Controller]: x=0.000, y=-0.000, theta=-0.004 rad
[INFO] [1762973116.634753974] [Controller]: Left->Right IR readings:  15.0   15.0   15.0   15.0   15.0   15.0   15.0
[INFO] [1762973117.631080984] [Controller]: x=0.000, y=-0.000, theta=-0.004 rad
[INFO] [1762973117.631643120] [Controller]: Left->Right IR readings:  15.0   15.0   15.0   15.0   15.0   15.0   15.0

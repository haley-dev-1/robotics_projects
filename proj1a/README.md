# Running ROS2 Project 1a

## 1. Starting Application Order
The general order to begin your work is to open three terminals. In each
terminal, and in this order, run the following:

### 1.1 Start Gazebo on ros domain 232 to avoid cross talk with robot :
```bash
ROS_DOMAIN_ID=232  \
ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py
```

  - If others are working at the same time in the lab, you may all need to
    agree to different numbers and replace 232 with that number.

### 1.2 Start the script you are modifying :
```bash
ROS_DOMAIN_ID=232  \
ros2 run drive_spiral main --ros-args -p spiral_scale:=1.0
```
---

## 2. Simulate button-presses for the robot:
  This is how you send message to the robot. When we move to the actual bot, we
  will literally press the buttons.

### 2.1 Left Button (button 1)
```bash
ROS_DOMAIN_ID=232  \
ros2 topic pub --once proj1/interface_buttons  \
irobot_create_msgs/msg/InterfaceButtons "{  \
  button_1: {is_pressed: true},  \
  button_power: {is_pressed: false},  \
  button_2: {is_pressed: false}  \
}"
```

### 2.2 Right Button (button 2)
```bash
ROS_DOMAIN_ID=232  \
ros2 topic pub --once  \
  proj1/interface_buttons  \
  irobot_create_msgs/msg/InterfaceButtons "{  \
    button_1: {is_pressed: false},  \
    button_power: {is_pressed: false},  \
    button_2: {is_pressed: true}  \
}"
```

### 2.3 Center Button (power button)
```bash
ROS_DOMAIN_ID=232  \
ros2 topic pub --once  \
proj1/interface_buttons  \
irobot_create_msgs/msg/InterfaceButtons "{  \
  button_1: {is_pressed: false},  \
  button_power: {is_pressed: true},  \
  button_2: {is_pressed: false}  \
}"
```

---

## 3. Expected responses before editing drive_spiral.py:
### 3.1 Left Button:
  - Feedback in the drive_sprial terminal on button press
  - Simulated robot begins driving in a CCW circle.
### 3.2 Right Button:
  - Feedback in the drive_sprial terminal on button press
  - Simulated robot begins driving in a CW circle.
### 3.3 Power Button:
  - Feedback in the drive_sprial terminal on button press
  - Simulated robot stops driving.

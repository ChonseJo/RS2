# RS2: UR3 Bottle Sorting System

This project integrates a UR3 robotic arm with ROS 2 to automate the sorting of bottles based on cap colors. 
It includes image processing, path planning, and gripper control to pick and place bottles in specific bins.

---

## 📁 Repository Structure

- **bottle_cap_vision**: Image processing and bottle cap color detection.
- **path_planning**: Path and motion planning for the UR3 arm.
- **servo_controller**: Custom control of gripper servos.
- **README.txt**: This project documentation file.

---

## 🧰 Requirements

- ROS 2 (Humble recommended)
- MoveIt 2
- UR3 robot and compatible driver (`ur_robot_driver`)
- Packages in this repository (e.g., `ur3_bottle_sorter`, `bottle_cap_vision`)

---

## ⚙️ Setup Instructions

### Launch UR3e
Testing with Hardware
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.0.194 launch_rviz:=false
```
Testing Offline
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=false
```

### Launch a custom UR3e MoveIt config in rviz
Launched with real world hardware
```bash
ros2 launch ur3_bottle_sorter ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true
```
Simulated in rviz
```bash
ros2 launch ur3_bottle_sorter bottle_sorter.launch.py ur_type:=ur3e simulation:=true
```

### Launch the bottle sorting system (and Servo Controller)
```bash
ros2 launch ur3_bottle_sorter bottle_sorter.launch.py ur_type:=ur3e
```

### Run the RealSense Camera
```bash
ros2 run realsense2_camera realsense2_camera_node
```

### Run the Bottle Cap Vision Package
```bash
ros2 run bottle_cap_vision realsense_capture_node
```


### Launch the Servo Controller
```bash
ros2 run servo_controller servo_serial_node

```
---

## 🎮 Usage

Once launched:
- Bottles are detected via the camera using bottle_cap_vision.
- Detected cap colors are classified (e.g., red, green, blue).
- The path_planning node plans a trajectory based on bottle type.
- The robot executes the move using MoveIt.
- The servo_controller communicates with the Arduino to actuate the gripper.

### Calling Home Position
```bash
ros2 service call /call_home std_srvs/srv/SetBool "{data: true}"
```

### Enabling Run Command
```bash
ros2 service call /toggle_run std_srvs/srv/SetBool "{data: true}"
```

---

## 📸 Preview
```
![Bottle Sorting Demo](<path to image goes here>)
```
---

## Notes
Grip height is 135mm from base of bottle box to base of gripper

---

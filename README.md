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
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3 robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=false
```

### Launch a custom UR3e MoveIt config in rviz
```bash
ros2 launch ur3_bottle_sorter ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true
```

### Launch the bottle sorting system
```bash
ros2 launch ur3_bottle_sorter ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true
```

---

## Notes
Grip height is 135mm from base of bottle box to base of gripper

---

## To Do
Setting positions via the gui and necessary ros2 nodes for visualisation
Design the test setup (where the bottle are placed/held, bins to dump the bottles, how are these placed/fixed to the table/lab equipment

# 📚 Pickasso Navigation Stack
This repository contains the ROS Navigation Stack setup for the Pickasso Robot, supporting simulation in Gazebo and deployment on TurtleBot3 Waffle Pi.

---

## 🚀 Features
- AMCL Localization
- move_base Path Planning with DWA Planner
- Obstacle Avoidance
- Gazebo Office Simulation Environment
- Real-World Ready Costmaps

---

## 📦 Installation
```bash
cd ~/catkin_ws/src
git clone https://github.com/AapseMatlb/pickasso_navigation_stack.git
cd ~/catkin_ws && catkin_make
source devel/setup.bash
```

---

## 📚 Usage
### Simulation:
```bash
roslaunch pickasso_navigation_stack simulation_world.launch
```

### Navigation:
```bash
roslaunch pickasso_navigation_stack navigation.launch
```

---

## 🛠️ Real-World Deployment Steps
1. Create map using SLAM.
2. Save map using map_saver.
3. Launch navigation using saved map.

---

# 📖 Additional Features (New)
- Autonomous Mapping and Exploration
- Obstacle Avoidance Using LiDAR
- Battery Monitoring and Position Saving
- Preloaded Office Map for SLAM Testing
- ROS Launch File for Easy Execution

---

## 📦 Dependencies

Ensure the following ROS packages are installed:

```bash
sudo apt install ros-<distro>-rospy ros-<distro>-geometry-msgs ros-<distro>-sensor-msgs ros-<distro>-nav-msgs
```
Replace `<distro>` with your ROS distribution (e.g., `noetic`).

---

## ▶️ How to Run

1. **Build the Workspace:**
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

2. **Run the Auto-Mapping Script:**
```bash
roslaunch pickasso_navigation_stack auto_mapping.launch
```

3. **View the Predefined Map:**
```bash
rosrun map_server map_server ~/catkin_ws/src/pickasso_navigation_stack/auto_mapping/maps/office_map.yaml
```

---

## 📖 Map Details

- **Resolution:** 0.05 m/pixel  
- **Origin:** [-5.0, -5.0, 0.0]  
- **Thresholds:**  
  - Occupied: 0.65  
  - Free: 0.196  

---

## 🛠️ Future Enhancements

- Integration with RViz and MoveIt!
- Multi-Room Mapping and Navigation
- Human-Robot Interaction for Permission-Based Actions

---

## 📄 License

This project is licensed under the MIT License.
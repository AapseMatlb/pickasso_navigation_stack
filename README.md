# Pickasso Navigation Stack

This repository contains the ROS Navigation Stack setup for the Pickasso Robot, supporting simulation in Gazebo and deployment on TurtleBot3 Waffle Pi.

## 🚀 Features
- AMCL Localization
- move_base Path Planning with DWA Planner
- Obstacle Avoidance
- Gazebo Office Simulation Environment
- Real-World Ready Costmaps

## 📦 Installation
```bash
cd ~/catkin_ws/src
git clone https://github.com/AapseMatlb/pickasso_navigation_stack.git
cd ~/catkin_ws && catkin_make
source devel/setup.bash
```

## 📚 Usage
- **Simulation:**  
  ```bash
  roslaunch pickasso_navigation_stack simulation_world.launch
  ```
- **Navigation:**  
  ```bash
  roslaunch pickasso_navigation_stack navigation.launch
  ```

## 🛠️ Real-World Deployment Steps
1. Create map using SLAM.
2. Save map using `map_saver`.
3. Launch navigation using saved map.

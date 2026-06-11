# TIAGo Warehouse Robot - TTTC 2343 Group 15

Autonomous warehouse robot simulation using TIAGo on ROS Noetic + Gazebo.

## Packages

### tiago_nav_ui (Week 9)
Multi-point navigation with UI control panel.
- 3 navigation checkpoints (Delivery, Processing, Pickup zones)
- Obstacle avoidance via move_base
- tkinter UI with real-time robot status

### tiago_aruco_task (Week 10)
ArUco marker scanning at task stations.
- 3 ArUco markers spawned at checkpoints
- Proximity-based detection
- Scan results displayed in navigation UI

## Launch

Terminal 1:
LIBGL_ALWAYS_SOFTWARE=1 roslaunch tiago_2dnav_gazebo tiago_navigation.launch public_sim:=true robot:=steel world:=small_warehouse

Terminal 2:
roslaunch tiago_nav_ui full_demo.launch

## Map Files
Located in map_files/ - small_warehouse environment.

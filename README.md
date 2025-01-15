# TurtleBot3 with RRT Path Planning and SLAM

This repository demonstrates how to set up and run a TurtleBot3 in a simulated environment using ROS 2 Humble, integrating SLAM, navigation, and RRT-based path planning.

## Prerequisites

Ensure you have the following installed and set up:

- **ROS 2 Humble**
- TurtleBot3 packages
- Python 3.x
- `slam_toolbox` and `nav2` packages
- `RRT.py` script (included in this repository)

## Steps to Run the Project

### 1. Launch the TurtleBot3 Simulation

Start the simulation environment for TurtleBot3.

```bash
ros2 launch autobot_recog warehouse_launch.launch.py
```

You can run with any map or world

### 2. Launch SLAM with Gmapping

Use the `slam_toolbox` package to perform SLAM (Simultaneous Localization and Mapping).

```bash
ros2 launch slam_toolbox online_async_launch.py
```

### 3. Open RViz for Visualization

Launch RViz to visualize the robot's surroundings and SLAM progress:

1. Open RViz.
2. Set the **Scan** and **Map** topics to visualize the LiDAR scans and generated map.
3. Add a **Pose Display** to set the robot's pose manually if required.

### 4. Save the Generated Map

Save the map to your desired location. The map will be saved in the current directory unless specified otherwise.

```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

This command saves two files: `my_map.pgm` and `my_map.yaml`.

### 5. Launch the Navigation Stack

Start the Navigation Stack using the saved map.

```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true map:=my_map.yaml
```

### 6. Run the RRT Path Planning Script

Execute the RRT (Rapidly-exploring Random Tree) path planning script to compute the robot's path.

```bash
python3 RRT.py
```

### 7. Additional Notes

- Ensure the map is saved in the same directory where the `navigation_launch.py` file is being run.
- Modify the `RRT.py` script to include your start and goal positions as required.


## Troubleshooting

1. **RViz not displaying topics:** Check if the correct topics are being subscribed to in RViz.
2. **Navigation stack issues:** Verify the map file path in the `navigation_launch.py` arguments.
3. **RRT script errors:** Ensure all dependencies for the script are installed and the start/goal positions are valid.

## Acknowledgments

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [SLAM Toolbox](https://navigation.ros.org/setup_guides/slam/slam_toolbox/)

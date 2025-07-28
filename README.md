# my_tb3_sim
This repository contains the simulation code, ROS 2 nodes, and experimental setup used in the paper:  
"Generation of Critical Information and Sharing Mechanism for Multi-Robot Mission Success" in *IEEE Access, 2025*

## Overview
This study evaluates a real-time information sharing mechanism between two TurtleBot3 robots under different QoS conditions.  
The repository includes:
- SLAM + Nav2 setup for two robots
- Obstacle detection and reaction logger
- ROS 2 QoS profile test scripts
- Gazebo Classic simulation world

## Environment
- **Ubuntu 22.04**
- **ROS 2 Humble**
- **Gazebo Classic**
- **TurtleBot3 Burger**
- Python 3.10+

## Directory Structure
my_tb3_sim/
├── launch/ # Launch files for robot systems and experiment
├── config/ # QoS profiles and Nav2 parameters
├── urdf/ # Simplified TB3 URDF
├── worlds/ # Warehouse-like Gazebo world
├── my_tb3_sim/ # ROS 2 Python nodes (publisher, logger, marker)
├── test/ # Lint tests
├── setup.py / package.xml # ROS 2 package metadata
└── README.md

1. Build
bash
cd ~/ros2_sim  # or your workspace
colcon build --packages-select my_tb3_sim
source install/setup.bash

2. Launch TB3-1 and TB3-2
ros2 launch my_tb3_sim tb3_1_system.launch.py
ros2 launch my_tb3_sim tb3_2_system.launch.py

3. Run experiment and logger
ros2 launch my_tb3_sim run_experiment.launch.py
# OR manually:
python3 my_tb3_sim/obstacle_info_publisher.py
python3 my_tb3_sim/reaction_logger.py


QoS Profiles
| Condition | Reliability | Durability | History   | Depth |
| --------- | ----------- | ---------- | --------- | ----- |
| A         | Reliable    | Volatile   | Keep Last | 10    |
| ...       | ...         | ...        | ...       | ...   |

Output
Experiment logs are stored via log_results.py and include:
Message delay
Reaction time
Message drop rate
Obstacle-aware path change


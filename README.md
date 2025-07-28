# my_tb3_sim: Simulation guide
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
- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic
- TurtleBot3 Burger
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

# 1. Build
bash
cd ~/ros2_sim  # or your workspace
colcon build --packages-select my_tb3_sim
source install/setup.bash

# 2. Launch TB3-1 and TB3-2
ros2 launch my_tb3_sim tb3_1_system.launch.py
ros2 launch my_tb3_sim tb3_2_system.launch.py

# 3. Run experiment and logger
ros2 launch my_tb3_sim run_experiment.launch.py
# OR manually:
python3 my_tb3_sim/obstacle_info_publisher.py
python3 my_tb3_sim/reaction_logger.py

# QoS Profiles
| Profile | Reliability | Durability | History  | Depth   | Network         |
| ------- | ----------- | ---------- | -------- | ------- | --------------- |
| A       | Reliable    | Volatile   | KeepLast | 10      | None            |
| B       | BestEffort  | Volatile   | KeepLast | 10      | None            |
| C       | Reliable    | Transient  | KeepLast | 10      | None            |
| D–G     | Mixed       | Varying    | Varying  | Varying | With delay/loss |

# Output
Experiment logs are stored via log_results.py and include:
Message delay, Reaction time, Message drop rate, Obstacle-aware path change

------------------------------------------------------------------------------------------------------------------------------------------
# ros2_real: Manual Execution Guide (for Real-Robot Deployment)
For researchers deploying this on Yahboom Pi5-based mobile robots, below are the actual commands used to launch each module step by step.

# ROS2 QoS-Based Multi-Robot Experiment (ros2_sim)
This repository provides a complete experimental setup for evaluating multi-robot cooperation using ROS 2 Humble on Yahboom Pi5 robots. 
It investigates how various QoS conditions impact obstacle sharing, map synchronization, and goal navigation performance over DDS.

## Experimental Scenario
- Robot 1 (Leader): SLAM + publishes `/obstacle_info`, `/map_update`, `/goal_pose`
- Robot 2 (Follower): Receives shared info, replans path, updates map, navigates to target
- QoS profiles (A~G) and network disturbance (delay/loss) can be applied

## Directory Structure
ros2_sim/
├── launch/                     # Launch files for each robot and RViz
├── config/                     # QoS profiles and nav2 configs
├── scripts/                    # Pub/sub nodes, logger, QoS loader, etc.
├── maps/                       # Saved map from SLAM
├── data/                       # Experiment result CSVs
├── run_experiment.sh          # Batch run per QoS condition
├── tc_netem_runner.sh         # Network disturbance injector

# Option 1: Automated Execution (Recommended)
# Run all steps for QoS Condition A (A ~ G)
bash scripts/run_experiment.sh A
Load qos_profile_A.yaml
Launch SLAM/NAV2 + pub/sub + logger
Simulate network delay/loss (if needed)
Save result as data/condition_A.csv

# Option 2: Manual Execution (For Real Robot Testing)
Used during real-world deployment on Yahboom Pi5
## Robot 1 (SLAM + Publish)
# 1. Start SLAM
ros2 launch slam_toolbox online_async_launch.py namespace:=robot1 use_sim_time:=false params_file:=config/nav2_params_pi5_1.yaml
# 2. Start obstacle_info publisher
ros2 run ros2_sim obstacle_info_publisher.py
# 3. Start map update publisher
ros2 run ros2_sim map_update_publisher.py
# 4. Start goal pose publisher
ros2 run ros2_sim goal_pose_publisher.py

## Robot 2 (Subscribe + Navigate)
# 1. Start Nav2 stack
ros2 launch nav2_bringup navigation_launch.py namespace:=robot2 use_sim_time:=false params_file:=config/nav2_params_pi5_2.yaml
# 2. Start obstacle_info subscriber
ros2 run ros2_sim obstacle_info_subscriber.py
# 3. Start map update subscriber
ros2 run ros2_sim map_update_subscriber.py
# 4. Start goal pose subscriber
ros2 run ros2_sim goal_pose_subscriber.py

# Optional Logging and Evaluation
# Logging script
ros2 run ros2_sim evaluation_logger.py

# Scenario2 Multi-Robot Exploration Simulation (Gazebo Classic)

본 프로젝트는 TurtleBot3 기반 ROS2 시스템에서 다중 로봇 협력 탐색을 수행하고,
우선순위 기반 실시간 정보 공유와 맵 병합을 통해 효율적인 미션 수행을 목표로 합니다.

## 💻 구성 요소
- ROS2 Humble + Gazebo Classic 11
- `slam_toolbox`, `nav2`, `multirobot_map_merge`
- 장애물/위험지역 감지 및 실시간 공유
- QoS 기반 네트워크 최적화
- Auction-based Task Allocation (기초 버전)
- 실험 자동화 + 성능 로깅

## 🚀 실행 방법
```bash
ros2 launch my_tb3_sim scenario2_experiment_classic.launch.py


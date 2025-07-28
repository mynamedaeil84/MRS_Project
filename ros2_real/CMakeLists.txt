cmake_minimum_required(VERSION 3.5)
project(my_tb3_sim)

# 기본적인 빌드 조건
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros2_control REQUIRED)


# install launch, config, world, urdf
install(DIRECTORY 
        launch/
        config/
        rviz/
        worlds/
        DESTINATION share/${PROJECT_NAME})
        
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf)
   
        
ament_package()

cmake_minimum_required(VERSION 3.8)
project(robot_car)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

add_executable(imu_broadcaster src/imu_broadcaster.cpp)
ament_target_dependencies(imu_broadcaster PUBLIC rclcpp sensor_msgs tf2 tf2_ros)

install(TARGETS imu_broadcaster
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY description launch config world meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/drive_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/wall_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

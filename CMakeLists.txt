cmake_minimum_required(VERSION 3.8)
project(minibex_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_ros2_control_xacro test/test_ros2_control_xacro.py)
endif()

ament_package()

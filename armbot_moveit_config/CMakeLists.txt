cmake_minimum_required(VERSION 3.22)
project(armbot_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

ament_package()
add_executable(mtc_node src/mtc_node.cpp)
target_include_directories(mtc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mtc_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  mtc_node
  moveit_task_constructor_core
  rclcpp
)

install(TARGETS mtc_node
  DESTINATION lib/${PROJECT_NAME})

  install(
    PROGRAMS 
    scripts/cuboiddetector.py
    DESTINATION lib/${PROJECT_NAME}
  )

install(
  DIRECTORY 
  launch
  config
  panda_description
  DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

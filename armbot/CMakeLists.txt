cmake_minimum_required(VERSION 3.8)
project(armbot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_python_install_package(${PROJECT_NAME})


install(
  DIRECTORY
  world
  models
  launch
  armbot 
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
 
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

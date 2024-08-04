cmake_minimum_required(VERSION 3.8)
project(ur_moveit_servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  sensor_msgs
  control_msgs
  moveit_msgs
  geometry_msgs
)

## copy config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME} )

## copy launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME} )

# Add an executable for using a controller (Component)
add_library(ur_joystick_input SHARED src/JoyToServoPub_ur10e.cpp)
ament_target_dependencies(ur_joystick_input ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(ur_joystick_input "ur_moveit_servo::JoyToServoPub")

install(TARGETS
  ur_joystick_input
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

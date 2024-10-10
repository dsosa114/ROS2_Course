cmake_minimum_required(VERSION 3.8)
project(mcr_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_warehouse REQUIRED)

add_executable(xarm_moveit_basic src/xarm_moveit_basic.cpp)

# target_link_libraries(xarm_moveit_basic moveit_ros_warehouse)

target_include_directories(xarm_moveit_basic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(xarm_moveit_basic PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  xarm_moveit_basic
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
  "moveit_ros_warehouse"
)

install(TARGETS xarm_moveit_basic
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
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

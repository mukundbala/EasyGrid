cmake_minimum_required(VERSION 3.8)
project(easy_grid_ros)



if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(easy_grid_core REQUIRED)

add_executable(easy_grid_demo
  src/easy_grid_demo_node.cpp
  src/easy_grid_demo.cpp
)

target_include_directories(easy_grid_demo
    PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

target_link_libraries(easy_grid_demo easy_grid_core::easy_grid)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install rviz files
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install include
install(DIRECTORY include
  DESTINATION include/${PROJECT_NAME}
)

# Install executables
install(TARGETS
  easy_grid_demo
  DESTINATION lib/${PROJECT_NAME})


ament_target_dependencies(easy_grid_demo
  rclcpp
  geometry_msgs
  visualization_msgs
  nav_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
)

ament_export_dependencies(
  rclcpp
  geometry_msgs
  visualization_msgs
  nav_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
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

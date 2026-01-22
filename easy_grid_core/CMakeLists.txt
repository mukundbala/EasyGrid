cmake_minimum_required(VERSION 3.16)
project(easy_grid_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# ----------------------------
# Header-only INTERFACE target
# ----------------------------
add_library(easy_grid_core INTERFACE)

target_include_directories(easy_grid_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(easy_grid_core INTERFACE
  Eigen3::Eigen
)

# ----------------------------
# Install
# ----------------------------
# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install target + export
install(TARGETS easy_grid_core
  EXPORT export_easy_grid_core
)

# Install CMake export files 
install(
  EXPORT export_easy_grid_core
  DESTINATION share/${PROJECT_NAME}/cmake
)

# ----------------------------
# Export
# ----------------------------
ament_export_targets(export_easy_grid_core)
ament_export_dependencies(Eigen3)
ament_export_include_directories(include)

# ----------------------------
# Linting
# ----------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

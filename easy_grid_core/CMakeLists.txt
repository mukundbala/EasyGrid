cmake_minimum_required(VERSION 3.8)
project(easy_grid_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# Header only easy_grid library
add_library(easy_grid INTERFACE)
target_include_directories(easy_grid INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(easy_grid INTERFACE
  Eigen3::Eigen
)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS easy_grid
  EXPORT export_easy_grid
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_easy_grid)
ament_export_dependencies(ament_cmake Eigen3)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Skip copyright and cpplint for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(easy_grid_metadata_test
    test/easy_grid_metadata_test.cpp
  )
  target_link_libraries(easy_grid_metadata_test
    easy_grid
  )

  ament_add_gtest(easy_grid_constructor_test
    test/easy_grid_constructor_test.cpp
  )
  target_link_libraries(easy_grid_constructor_test
    easy_grid
  )

  ament_add_gtest(easy_grid_conversion_test
    test/easy_grid_conversion_test.cpp
  )
  target_link_libraries(easy_grid_conversion_test easy_grid)
  
  ament_add_gtest(easy_grid_bounds_test
    test/easy_grid_bounds_test.cpp
  )
  target_link_libraries(easy_grid_bounds_test easy_grid)
  
  ament_add_gtest(easy_grid_cell_access_test
    test/easy_grid_cell_access_test.cpp
  )
  target_link_libraries(easy_grid_cell_access_test easy_grid)
  
  # ament_add_gtest(easy_grid_iterator_test
  #   test/easy_grid_iterator_test.cpp
  # )
  # target_link_libraries(easy_grid_iterator_test easy_grid)
  
  # ament_add_gtest(easy_grid_polygon_test
  #   test/easy_grid_polygon_test.cpp
  # )
  # target_link_libraries(easy_grid_polygon_test easy_grid)

endif()

ament_package()
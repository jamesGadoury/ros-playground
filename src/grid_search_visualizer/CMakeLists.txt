cmake_minimum_required(VERSION 3.8)
project(grid_search_visualizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(SFML COMPONENTS graphics REQUIRED)
find_package(rclcpp REQUIRED)
find_package(grid_search_interfaces REQUIRED)

add_executable(grid_search_visualizer src/grid_search_visualizer.cpp)
ament_target_dependencies(grid_search_visualizer rclcpp grid_search_interfaces)
target_link_libraries(grid_search_visualizer sfml-graphics)

target_include_directories(grid_search_visualizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(grid_search_visualizer PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS grid_search_visualizer
  DESTINATION lib/${PROJECT_NAME})
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

cmake_minimum_required(VERSION 3.8)
project(amr_order_optimizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msg REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_order_optimizer test/test_order_optimizer.cpp)
  # target_include_directories(test_order_optimizer PUBLIC
  #   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  #   $<INSTALL_INTERFACE:include>
  # )
  #target_link_libraries(${PROJECT_NAME}_tutorial_test name_of_local_library)
  ament_target_dependencies(test_order_optimizer rclcpp std_msgs custom_msg geometry_msgs yaml-cpp visualization_msgs)
  target_link_libraries(test_order_optimizer yaml-cpp)
endif()

# Declaring C++ executables
add_executable(OrderOptimizer src/order_optimizer.cpp)
ament_target_dependencies(OrderOptimizer rclcpp std_msgs custom_msg geometry_msgs yaml-cpp visualization_msgs)
target_link_libraries(OrderOptimizer yaml-cpp)

# Install the executable
install(TARGETS
  OrderOptimizer
  test_order_optimizer
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install python files
install(PROGRAMS
  launch/order_optimizer_launch.py  # Python launch file here
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()

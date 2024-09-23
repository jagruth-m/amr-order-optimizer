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

# Declaring C++ executables
add_executable(OrderOptimizer src/order_optimizer.cpp)
ament_target_dependencies(OrderOptimizer rclcpp std_msgs custom_msg geometry_msgs yaml-cpp visualization_msgs)
target_link_libraries(OrderOptimizer yaml-cpp)

# Install the executable
install(TARGETS
  OrderOptimizer
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install python files
install(PROGRAMS
  launch/order_optimizer_launch.py 
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(SOURCES src/order_optimizer.cpp)
  add_library(optimizer_test SHARED ${SOURCES})
  
  ament_add_gtest(test_order_optimizer test/test_order_optimizer.cpp)
  ament_target_dependencies(optimizer_test rclcpp std_msgs custom_msg geometry_msgs yaml-cpp visualization_msgs)
  target_link_libraries(test_order_optimizer optimizer_test yaml-cpp)

endif()


ament_package()

cmake_minimum_required(VERSION 3.8)
project(car_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(drive_msgs REQUIRED)

include_directories(include)

add_executable(drive_parameters_multiplexer
  src/drive_parameters_source.cpp
  src/drive_parameters_multiplexer.cpp
)
ament_target_dependencies(drive_parameters_multiplexer
  rclcpp
  std_msgs
  drive_msgs
)

add_executable(car_controller
  src/car_controller.cpp
)
ament_target_dependencies(car_controller
  rclcpp
  std_msgs
  geometry_msgs
  drive_msgs
)

install(TARGETS
  drive_parameters_multiplexer
  car_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()

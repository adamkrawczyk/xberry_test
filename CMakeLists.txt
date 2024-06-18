cmake_minimum_required(VERSION 3.8)
# cmake_policy(SET CMP0148 NEW)
project(xberry_test)

# Add compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  # Ensure it's found if you're using it
find_package(rosidl_default_generators REQUIRED)

# QT dependencies
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Gui REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(srv_files
  "srv/SetFloat.srv"
  "srv/SetFrequency.srv"
  "srv/SetTemperature.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
)

include_directories(
  include
)

# ROS2 nodes
add_executable(sensor_float src/sensor_float.cpp)
add_executable(sensor_temperature src/sensor_temperature.cpp)
ament_target_dependencies(sensor_float rclcpp std_msgs)
ament_target_dependencies(sensor_temperature rclcpp sensor_msgs)

# QT
add_executable(qt_subscriber_node src/qt_subscriber_node.cpp src/main.cpp)
ament_target_dependencies(qt_subscriber_node rclcpp std_msgs sensor_msgs)
target_link_libraries(qt_subscriber_node Qt5::Core Qt5::Widgets Qt5::Gui)

rosidl_target_interfaces(sensor_float
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

rosidl_target_interfaces(sensor_temperature
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(TARGETS
  sensor_float
  sensor_temperature
  qt_subscriber_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.5)
project(robot_control)

# Set the C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
#find_package(ur16_arc_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(repair_interface REQUIRED)


# Include your package's own include directory (if you have one)
include_directories(include)

# Set the external include directory path where Serial.h (or SerialPort.h) is located.
# Change the path below to your actual location.
#set(EXTERNAL_SERIAL_PATH "/home/robotics/robot_control/SerialArduino/Source")
set(EXTERNAL_SERIAL_PATH "/home/robotics/development/vacum_ws/Serial/Source")
include_directories(${EXTERNAL_SERIAL_PATH})

# Add the executable target.
# Make sure to list all source files that provide definitions (like Serial.cpp).
add_executable(controls
  src/controls.cpp
  ${EXTERNAL_SERIAL_PATH}/Serial.cpp
)

add_executable(joy_to_cmd
  src/joy_to_cmd
)

add_executable(line_follower src/line_follower.cpp)


# Alternatively, if you want to use target_include_directories for your target:
target_include_directories(controls PUBLIC ${EXTERNAL_SERIAL_PATH})

# Link against the required libraries
ament_target_dependencies(controls
  rclcpp
  geometry_msgs
  Boost
  repair_interface
)
ament_target_dependencies(joy_to_cmd
  rclcpp
  geometry_msgs
  sensor_msgs
)

ament_target_dependencies(line_follower
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  cv_bridge 
  OpenCV
)

# Link pthread if needed
target_link_libraries(controls pthread)

# Install the executable
install(TARGETS
  controls
  joy_to_cmd
  line_follower
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()

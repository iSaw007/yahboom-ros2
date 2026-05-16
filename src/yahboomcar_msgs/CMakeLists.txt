cmake_minimum_required(VERSION 3.8)
project(yahboomcar_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# Generate the interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Dock.action"
  DEPENDENCIES action_msgs builtin_interfaces std_msgs
)

ament_package()

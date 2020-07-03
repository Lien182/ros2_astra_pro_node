
cmake_minimum_required(VERSION 3.5)
project(astra_pro)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(include)
include_directories(Thirdparty/ReconROS)
include_directories(/opt/ros/dashing/include)
include_directories(Thirdparty/AstraSDK-v2.1.1/include)

LINK_DIRECTORIES(Thirdparty/AstraSDK-v2.1.1/lib)
LINK_DIRECTORIES(/opt/ros/dashing/lib/)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_executable(astra_pro_node src/main.cpp )

target_link_libraries( astra_pro_node PRIVATE astra astra_core astra_core_api Threads::Threads rcl rcutils rclcpp std_msgs__rosidl_typesupport_cpp sensor_msgs__rosidl_typesupport_cpp )


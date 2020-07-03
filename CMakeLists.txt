
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

add_library(astra_pro SHARED src/astra_pro.cpp)
target_link_libraries(astra_pro ${ament_LIBRARIES} ${OpenCV_LIBRARIES})
ament_target_dependencies(astra_pro
  ${DEPENDENCIES}
)

rclcpp_components_register_nodes(astra_pro "${PROJECT_NAME}::ImagePublisher")
set(node_plugins "${node_plugins}${PROJECT_NAME}::ImagePublisher;$<TARGET_FILE:ImagePublisher>\n")

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.h")

add_executable(astra_pro_node src/astra_pro_node.cpp)
target_link_libraries(astra_pro_node ${ament_LIBRARIES} astra_pro astra astra_core astra_core_api Threads::Threads)

ament_target_dependencies(astra_pro_node
  ${DEPENDENCIES})

install(TARGETS astra_pro_node
        DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS astra_pro
        DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(astra_pro_node src/main.cpp )

target_link_libraries( astra_pro_node PRIVATE astra astra_core astra_core_api Threads::Threads rcl rcutils rclcpp std_msgs__rosidl_typesupport_cpp sensor_msgs__rosidl_typesupport_cpp )


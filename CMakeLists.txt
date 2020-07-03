cmake_minimum_required(VERSION 3.10)

set(CMAKE_BUILD_TYPE Debug)

#include(cmake/FindAstra.cmake)

# set the project name and version
project(astra_pro_node)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(include)
include_directories(Thirdparty/AstraSDK-v2.1.1/include)

LINK_DIRECTORIES(Thirdparty/AstraSDK-v2.1.1/lib)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)


add_executable(astra_pro_node src/main.cpp)

target_link_libraries( astra_pro_node PRIVATE astra astra_core astra_core_api Threads::Threads)


cmake_minimum_required(VERSION 3.10)
project(trybug)

set(CMAKE_CXX_STANDARD 11)

find_package(cartographer REQUIRED)
find_package(Boost 1.55.0 REQUIRED COMPONENTS system filesystem)

include_directories(include/
        /usr/include/eigen3/
        ${Boost_INCLUDE_DIRS})

add_executable(trybug main.cpp
        include/cartographer_ros/time_conversion.cc
        include/cartographer_ros/msg_conversion.cc
        include/cartographer_ros/tf_bridge.cc
        include/cartographer_ros/map_builder_bridge.cc)

target_link_libraries(${PROJECT_NAME} PUBLIC cartographer ${testlaser_LIBRARIES} ${Boost_LIBRARIES} pthread)
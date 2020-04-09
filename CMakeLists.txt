cmake_minimum_required(VERSION 3.0.0)
project(cPlusDebug)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 11)

string(TIMESTAMP VERSION "%Y-%m-%d %H:%M")
add_definitions(-DVERSION="${VERSION}")

add_definitions( -D__PX4_POSIX_SITL_SIMULATE )
set(SRC  ${PROJECT_SOURCE_DIR}/c_plus.cpp
         ${PROJECT_SOURCE_DIR}/CChain.cpp)
add_executable(exe ${SRC})

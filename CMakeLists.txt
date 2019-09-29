cmake_minimum_required(VERSION 3.5)
project(indemind_collect_data)

set(CMAKE_BUILD_TYPE  Release)
set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
link_directories(${PROJECT_SOURCE_DIR}/driver_x86_64)

include_directories(${OPENCV_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/driver_x86_64/include)
include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(indemind_collect_data src/main.cpp)
target_link_libraries(indemind_collect_data ${OpenCV_LIBS} usbdriver pthread)

add_executable(test_load_params src/test_param_manager.cpp)

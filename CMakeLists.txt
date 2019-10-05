cmake_minimum_required(VERSION 3.0.0)
project(hwsfm VERSION 0.1.0)

add_definitions(-std=c++11)
set(OPENCV_ENABLE_NONFREE)
find_package(OpenCV 3.4 REQUIRED)

include_directories("/usr/include/eigen3" include)

add_library(setting src/setting.cpp)
target_link_libraries(setting ${OpenCV_LIBS})

add_library(hwsfm src/hwsfm.cpp src/frame.cpp src/mappoint.cpp)
target_link_libraries(hwsfm ${OpenCV_LIBS})

add_executable(main src/main.cpp)
target_link_libraries(main 
${OpenCV_LIBS}
setting
hwsfm
)


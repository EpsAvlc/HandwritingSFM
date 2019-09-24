cmake_minimum_required(VERSION 3.0.0)
project(hwsfm VERSION 0.1.0)

find_package(OpenCV REQUIRED)
include_directories(include )

add_library(setting src/setting.cpp)
target_link_libraries(setting ${OpenCV_LIBS})

add_executable(hwsfm src/main.cpp)
target_link_libraries(hwsfm 
${OpenCV_LIBS}
setting
)


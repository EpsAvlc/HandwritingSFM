cmake_minimum_required(VERSION 3.0.0)
project(hwsfm VERSION 0.1.0)

add_definitions(-std=c++11)
set(OPENCV_ENABLE_NONFREE)
find_package(OpenCV 3.4 REQUIRED)

find_package(Pangolin 0.4 REQUIRED)

include_directories("/usr/include/eigen3" 
    include
    ${Pangolin_INCLUDE_DIRS})

add_library(setting src/setting.cpp)
target_link_libraries(setting ${OpenCV_LIBS})

add_library(frame src/frame.cpp)
target_link_libraries(frame ${OpenCV_LIBS})

add_library(mappoint src/mappoint.cpp)
target_link_libraries(mappoint ${OpenCV_LIBS})

add_library(viewer src/viewer.cpp)
target_link_libraries(viewer
${Pangolin_LIBRARIES}
)

add_library(hwsfm src/hwsfm.cpp)
target_link_libraries(hwsfm 
    frame
    mappoint
    viewer
    ${OpenCV_LIBS}
    )

add_executable(main src/main.cpp)
target_link_libraries(main 
${OpenCV_LIBS}
setting
hwsfm
)


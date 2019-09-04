cmake_minimum_required(VERSION 3.0.0)
project(hwsfm VERSION 0.1.0)

find_package(OpenCV REQUIRED)

add_executable(hwsfm src/main.cpp)
target_link_libraries(hwsfm ${OpenCV_LIBS})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

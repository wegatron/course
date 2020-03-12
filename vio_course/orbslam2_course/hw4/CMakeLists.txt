cmake_minimum_required(VERSION 2.8)
project(vo_simulate)

add_definitions(-DWORKSPACE_DIR="${CMAKE_SOURCE_DIR}")

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -march=native")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -march=native")

include($ENV{HOME}/usr/share/cmake_modules/cmake_self.cmake)

MESSAGE("PROJECT_SOURCE_DIR " ${PROJECT_SOURCE_DIR})

#opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3  REQUIRED)

find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIR})
include_directories(${G2O_INCLUDE_DIR}/g2o/EXTERNAL/csparse)

include_directories(${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/src)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OPENCV_INCLUDE_DIR})

add_executable(vo_simulate  demo/main.cpp src/two_view_geometry.cpp src/utils.cpp src/optimizer.cpp)
target_link_libraries(vo_simulate ${OpenCV_LIBS} ${G2O_LIBRARIES})

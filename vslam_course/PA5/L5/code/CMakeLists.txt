cmake_minimum_required(VERSION 3.10)
project(pa5)

set(CMAKE_CXX_STANDARD 11)

#opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(
        /opt/ros/melodic/include
        ${EIGEN3_INCLUDE_DIR})

find_package(Pangolin REQUIRED)

add_executable(compute_orb
computeORB.cpp)

add_executable(e2rt
E2Rt.cpp)

add_executable(GN-BA GN-BA.cpp)

add_executable(compare_trajectory compare_trajectory.cpp)

target_link_libraries(compare_trajectory
  ${Pangolin_LIBRARIES})

target_link_libraries(GN-BA
${OpenCV_LIBS})

target_link_libraries(compute_orb
${OpenCV_LIBS})

target_link_libraries(e2rt
  ${OpenCV_LIBS})

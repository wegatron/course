cmake_minimum_required(VERSION 3.10)
project(vslam_pa6)

set(CMAKE_CXX_STANDARD 11)

#eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

#opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Pangolin REQUIRED)

# sophus
include_directories(/opt/ros/melodic/include/)

add_executable(optical_flow
optical_flow.cpp)

add_executable(direct direct_method.cpp)

add_executable(disparity disparity.cpp)

target_link_libraries(optical_flow
${OpenCV_LIBS}
)

target_link_libraries(direct
  ${Pangolin_LIBRARIES}
  ${OpenCV_LIBS})

target_link_libraries(disparity
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
)

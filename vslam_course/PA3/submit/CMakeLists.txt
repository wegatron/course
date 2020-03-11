cmake_minimum_required(VERSION 3.10)
project(vslam_pa3)

set(CMAKE_CXX_STANDARD 14)

set(app_name vslam_pa3)

find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)


include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    /opt/ros/melodic/include
)

add_executable(draw_trajectory draw_trajectory.cpp)

add_executable(estimate_err estimate.cpp)

target_link_libraries(draw_trajectory
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
)
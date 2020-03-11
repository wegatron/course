cmake_minimum_required(VERSION 3.10)
project(cv_pa4)

set(CMAKE_CXX_STANDARD 11)

set(app_name cv_pa4)

#opencv
find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
)

add_executable(image_undistort
  undistort_image.cpp)

add_executable(disparity
  disparity.cpp)

add_executable(gn gaussnewton.cpp)

target_link_libraries(image_undistort
${OpenCV_LIBS}
)

target_link_libraries(disparity
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
)

target_link_libraries(gn
  ${OpenCV_LIBS}
)

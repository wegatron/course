cmake_minimum_required(VERSION 3.10)
project(ba)

set(CMAKE_CXX_STANDARD 11)

list(APPEND CMAKE_MODULE_PATH $ENV{HOME}/usr/share/cmake_modules)

# add_executable(direct_ba
#  directBA.cpp
# )

add_executable(ba
bundle_adj.cpp
)

#eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

#opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(SuiteSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

#G2O
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIR})
include_directories(${G2O_INCLUDE_DIR}/g2o/EXTERNAL/csparse)

find_package(Pangolin REQUIRED)

include_directories(/opt/ros/melodic/include/)

# target_link_libraries(direct_ba
#     ${OpenCV_LIBS}
#     ${Pangolin_LIBRARIES}
#     ${G2O_LIBRARIES}
# )

target_link_libraries(ba
${OpenCV_LIBS}
${Pangolin_LIBRARIES}
${G2O_LIBRARIES}
${CSPARSE_LIBRARY}
)

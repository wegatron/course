cmake_minimum_required(VERSION 3.10)
project(ground_seg)

include($ENV{HOME}/usr/share/cmake_modules/cmake_self.cmake)

# 设置编译输出的路径
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -march=native")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

set(app_name ground_seg)

add_definitions(-D_USE_MATH_DEFINES)

# find_package(Eigen3  REQUIRED)
# include_directories(${EIGEN3_INCLUDE_DIR})

#boost
#find_package(Boost COMPONENTS filesystem REQUIRED)
#add_definitions(-DBOOST_SYSTEM_NO_DEPRECATED)
#include_directories(${Boost_INCLUDE_DIRS})
#link_directories(${Boost_LIBRARY_DIRS})

#PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(ZSW_LIB REQUIRED)
include_directories(${ZSW_LIB_INCLUDE_DIR})

file(GLOB headers *.h *.hpp)
file(GLOB src *.cpp)

add_executable(${app_name}
${headers}
${src})

target_link_libraries(${app_name}
  ${ZSW_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

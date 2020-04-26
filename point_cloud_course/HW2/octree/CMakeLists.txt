cmake_minimum_required(VERSION 3.10)
project(wegatron_octree)

# 设置编译输出的路径
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -march=native")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -march=native")

#eigen
find_package(Eigen3  REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# python3
set(ENV{PATH} "$ENV{PATH};/home/wegatron/opt/anaconda3/envs/pc_course/bin")
find_package(PythonInterp 3 REQUIRED)
find_package(PythonLibs 3 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

#boost
find_package(Boost 1.65 COMPONENTS python3 numpy3 REQUIRED)
add_definitions(-DBOOST_SYSTEM_NO_DEPRECATED)
add_definitions(-DBOOST_PYTHON_STATIC_LIB) # 这里必须使用静态库
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

file(GLOB headers *.h *.hpp)
file(GLOB src *.cpp)

PYTHON_ADD_MODULE(wegatron_octree ${headers} ${src})

# add_library(wegatron_octree ${headers} ${src})

target_link_libraries(wegatron_octree
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
  )

cmake_minimum_required(VERSION 3.10)
project(lidar_slam)

set(CMAKE_CXX_STANDARD 14)

set(app_name lidar_slam)

include($ENV{HOME}/usr/cmake_modules/cmake_self.cmake)
include($ENV{HOME}/usr/cmake_modules/common.cmake)

# 设置编译输出的路径
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin)

find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})

#eigen
include_directories(${EIGEN_INCLUDE_DIRS})

#boost
find_package(Boost COMPONENTS  filesystem REQUIRED)
add_definitions(-DBOOST_SYSTEM_NO_DEPRECATED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

#PCL
find_package(PCL 1.8 REQUIRED COMPONENTS common features filters kdtree ml octree registration io segmentation visualization)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

#for zsw lib
find_package(ZSW_LIB)
include_directories(${ZSW_LIB_INCLUDE_DIR})

# file(GLOB headers *.h *.hpp)
# file(GLOB src *.cpp)

# add_executable(surfel_extract
# ${headers}
# ${src}
# )

# target_link_libraries(surfel_extract
# ${PCL_LIBRARIES}
# ${VTK_LIBRARIES}
# ${Boost_LIBRARIES}
# ${ZSW_LIBRARIES}
# )

add_subdirectory(laser_segment)

# 可执行程序的后缀需要额外设置
set_target_properties(laser_segment PROPERTIES
DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
COMPILE_FLAGS "/MP")

# configure the project to set some debug parameters in VS
configure_file($ENV{HOME}/usr/vs.user.in laser_segment.vcxproj.user)

# 设置启动项目, 必须在最外层的CMAKELists.txt中设置
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT surfel_extract)

# 可执行程序安装的基本设置
# install(TARGETS ${app_name} RUNTIME DESTINATION .)
# install(CODE 
# "include(BundleUtilities)
# fixup_bundle(${CMAKE_INSTALL_PREFIX}/${app_name}${CMAKE_RELEASE_POSTFIX}.exe \"\" \"${path}\")")"
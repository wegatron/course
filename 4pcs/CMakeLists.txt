cmake_minimum_required(VERSION 3.10)
project(lasser3d_4pcs)

set(CMAKE_CXX_STANDARD 14)

set(app_name lasser3d_4pcs)

include($ENV{HOME}/usr/cmake_modules/cmake_self.cmake)
include($ENV{HOME}/usr/cmake_modules/common.cmake)

# 设置编译输出的路径
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin)

#eigen
include_directories(${EIGEN_INCLUDE_DIRS})

#boost
find_package(Boost COMPONENTS  filesystem REQUIRED)
add_definitions(-DBOOST_SYSTEM_NO_DEPRECATED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})

#PCL
find_package(PCL 1.8 REQUIRED COMPONENTS common features filters kdtree ml octree registration io segmentation visualization)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(ZSW_LIB)
include_directories(${ZSW_LIB_INCLUDE_DIR})

file(GLOB headers *.h *.hpp)
file(GLOB src *.cc)

add_executable(${app_name}
${headers}
${src}
remove_small_island.cpp
)

add_executable(demo_filtering
remove_small_island.h
remove_small_island.cpp
demo_io.h
demo_io.cc
fast_range_seg.cpp
fast_range_seg.h
demo.cpp
)

add_executable(gen_demo_data
gen_demo_data.cxx)


target_link_libraries(demo_filtering
${ZSW_LIBRARIES}
${PCL_LIBRARIES}
${VTK_LIBRARIES}
)

target_link_libraries(${app_name}
${ZSW_LIBRARIES}
${PCL_LIBRARIES}
${Boost_LIBRARIES}
${VTK_LIBRARIES}
)

target_link_libraries(gen_demo_data
${ZSW_LIBRARIES}
${PCL_LIBRARIES}
${Boost_LIBRARIES}
${VTK_LIBRARIES}
)

# 可执行程序的后缀需要额外设置
set_target_properties(${app_name} PROPERTIES
DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
COMPILE_FLAGS "/MP")

# configure the project to set some debug parameters in VS
configure_file(${CMAKE_SOURCE_DIR}/vs.user.in ${app_name}.vcxproj.user)
configure_file(${CMAKE_SOURCE_DIR}/vs.user.in demo_filtering.vcxproj.user)
configure_file(${CMAKE_SOURCE_DIR}/vs.user.in gen_demo_data.vcxproj.user)

# 设置启动项目, 必须在最外层的CMAKELists.txt中设置
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${app_name})

# # 可执行程序安装的基本设置
# install(TARGETS ${app_name} RUNTIME DESTINATION .)
# install(CODE 
# "include(BundleUtilities)
# fixup_bundle(${CMAKE_INSTALL_PREFIX}/${app_name}${CMAKE_RELEASE_POSTFIX}.exe \"\" \"${path}\")")"


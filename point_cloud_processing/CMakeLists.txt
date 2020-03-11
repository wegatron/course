cmake_minimum_required(VERSION 3.10)
project(pc_process)

set(CMAKE_CXX_STANDARD 14)

set(app_name pc_process)

if(WIN32)

include($ENV{HOME}/usr/cmake_modules/cmake_self.cmake)
include($ENV{HOME}/usr/cmake_modules/common.cmake)

# 设置编译输出的路径
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin)

endif(WIN32)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH};$ENV{HOME}/usr/cmake_modules)

#boost
find_package(Boost COMPONENTS  filesystem REQUIRED)
add_definitions(-DBOOST_SYSTEM_NO_DEPRECATED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

#PCL
find_package(PCL 1.8 REQUIRED COMPONENTS common features filters kdtree ml octree registration io segmentation visualization)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})

#for openmp
find_package(OpenMP)
if (OPENMP_FOUND)
message("openmp enabled!!!")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(ZSW_LIB)
include_directories(${ZSW_LIB_INCLUDE_DIR})

file(GLOB headers *.h *.hpp)
file(GLOB src *.cpp)

add_executable(demo_normal_gen demo_normal_gen.cxx)
add_executable(demo_outlier_remove demo_outlier_remove.cxx)
add_executable(demo_analysis_mapping demo_analysis_mapping.cxx)

set(ACTIVE_POJ demo_analysis_mapping)

target_link_libraries(demo_normal_gen
${ZSW_LIBRARIES}
${PCL_LIBRARIES}
${VTK_LIBRARIES}
)


target_link_libraries(demo_outlier_remove
${ZSW_LIBRARIES}
${PCL_LIBRARIES}
${VTK_LIBRARIES}
)

target_link_libraries(demo_analysis_mapping
${ZSW_LIBRARIES}
${PCL_LIBRARIES}
${VTK_LIBRARIES}
)

if(WIN32)
# 可执行程序的后缀需要额外设置
set_target_properties(demo_normal_gen PROPERTIES
DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
COMPILE_FLAGS "/MP")

set_target_properties(demo_outlier_remove PROPERTIES
DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
COMPILE_FLAGS "/MP")

# configure the project to set some debug parameters in VS
configure_file(${CMAKE_SOURCE_DIR}/vs.user.in demo_normal_gen.vcxproj.user)
configure_file(${CMAKE_SOURCE_DIR}/vs.user.in demo_outlier_remove.vcxproj.user)
configure_file(${CMAKE_SOURCE_DIR}/vs.user.in demo_analysis_mapping.vcxproj.user)

# 设置启动项目, 必须在最外层的CMAKELists.txt中设置
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${ACTIVE_POJ})

endif(WIN32)

# # 可执行程序安装的基本设置
# install(TARGETS ${app_name} RUNTIME DESTINATION .)
# install(CODE 
# "include(BundleUtilities)
# fixup_bundle(${CMAKE_INSTALL_PREFIX}/${app_name}${CMAKE_RELEASE_POSTFIX}.exe \"\" \"${path}\")")"

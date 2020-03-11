cmake_minimum_required(VERSION 3.10)
project(full_back_end)

set(CMAKE_CXX_STANDARD 14)

set(app_name full_back_end)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "$ENV{HOME}/usr/cmake_modules;/usr/lib/cmake/eigen3;/usr/lib/cmake/Ceres" )
include($ENV{HOME}/usr/cmake_modules/cmake_self.cmake)

# 设置编译输出的路径
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin)

#glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

#suitsparse
find_package(SuiteSparse REQUIRED)
include_directories(${CHOLMOD_INCLUDE_DIR})

#G2O
message(!!!!${G2O_INCLUDE_DIR})
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIR})
include_directories(${G2O_INCLUDE_DIR}/g2o/EXTERNAL/csparse)

#eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#vtk
find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})


file(GLOB headers *.h *.hpp)
file(GLOB src *.cpp)
add_executable(${app_name}
${headers}
${src}
)

target_link_libraries(${app_name}
        ${G2O_LIBRARIES}
        ${CHOLMOD_LIBRARY}
        ${VTK_LIBRARIES}
        ${GLOG_LIBRARIES}
        )

# 可执行程序的后缀需要额外设置
# set_target_properties(${app_name} PROPERTIES
# DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
# RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
# COMPILE_FLAGS "/MP")

# configure the project to set some debug parameters in VS
# configure_file(${CMAKE_SOURCE_DIR}/vs.user.in ${app_name}.vcxproj.user)

# 设置启动项目, 必须在最外层的CMAKELists.txt中设置
# set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${app_name})

# # 可执行程序安装的基本设置
# install(TARGETS ${app_name} RUNTIME DESTINATION .)
# install(CODE 
# "include(BundleUtilities)
# fixup_bundle(${CMAKE_INSTALL_PREFIX}/${app_name}${CMAKE_RELEASE_POSTFIX}.exe \"\" \"${path}\")")"

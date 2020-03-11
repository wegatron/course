cmake_minimum_required(VERSION 3.10)
project(pa2)

set(CMAKE_CXX_STANDARD 11)

set(app_name pa2)

include_directories("/usr/include/eigen3")

add_executable(pa2_1 use_decomposition.cpp)
add_executable(pa2_2 use_quaternion.cpp)

# target_link_libraries(${app_name}
# ...
# )

# 可执行程序的后缀需要额外设置
# set_target_properties(${app_name} PROPERTIES
# DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
# RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
# COMPILE_FLAGS "/MP")

# 可执行程序安装的基本设置
# install(TARGETS ${app_name} RUNTIME DESTINATION .)
# install(CODE 
# "include(BundleUtilities)
# fixup_bundle(${CMAKE_INSTALL_PREFIX}/${app_name}${CMAKE_RELEASE_POSTFIX}.exe \"\" \"${path}\")")"

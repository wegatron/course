cmake_minimum_required(VERSION 3.10)
project(orbslam_lib)

set(CMAKE_CXX_STANDARD 14)

file(GLOB headers *.h *.hpp)
file(GLOB src *.cc)
add_library(orbslam_lib
${headers}
${src}
)

add_executable(orb_extractor orb_extract_demo.cxx)

#opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
#${OpenCV_LIBS}

target_link_libraries(orbslam_lib
${OpenCV_LIBS}
)

target_link_libraries(orb_extractor
orbslam_lib
)



# # 可执行程序安装的基本设置
# install(TARGETS ${app_name} RUNTIME DESTINATION .)
# install(CODE 
# "include(BundleUtilities)
# fixup_bundle(${CMAKE_INSTALL_PREFIX}/${app_name}${CMAKE_RELEASE_POSTFIX}.exe \"\" \"${path}\")")"
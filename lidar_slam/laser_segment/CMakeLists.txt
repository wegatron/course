file(GLOB headers *.h *.hpp)
file(GLOB src *.cpp)

add_executable(laser_segment ${headers} ${src} demo.cxx)

target_link_libraries(laser_segment
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
  ${Boost_LIBRARIES}
  ${ZSW_LIBRARIES}
)
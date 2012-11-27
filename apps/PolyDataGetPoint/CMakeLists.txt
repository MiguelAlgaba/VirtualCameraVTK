cmake_minimum_required(VERSION 2.8)
 
PROJECT(PolyDataGetPoint)
 
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
 
add_executable(PolyDataGetPoint MACOSX_BUNDLE PolyDataGetPoint)
 
if(VTK_LIBRARIES)
  target_link_libraries(PolyDataGetPoint ${VTK_LIBRARIES})
else()
  target_link_libraries(PolyDataGetPoint vtkHybrid )
endif()

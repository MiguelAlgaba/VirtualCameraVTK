cmake_minimum_required(VERSION 2.8)
 
PROJECT(ReadPLY)
 
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
 
add_executable(ReadPLY MACOSX_BUNDLE ReadPLY)
 
target_link_libraries(ReadPLY ${VTK_LIBRARIES})

cmake_minimum_required(VERSION 2.8)
 
PROJECT(JPEGWriter)
 
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
 
add_executable(JPEGWriter MACOSX_BUNDLE JPEGWriter)
 
target_link_libraries(JPEGWriter ${VTK_LIBRARIES})

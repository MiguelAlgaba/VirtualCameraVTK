cmake_minimum_required(VERSION 2.8)
 
PROJECT(VirtualCameraVTK)
 
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
 
ADD_SUBDIRECTORY(apps)

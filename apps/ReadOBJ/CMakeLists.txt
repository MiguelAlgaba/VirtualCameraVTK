add_executable(ReadOBJ MACOSX_BUNDLE ReadOBJ)
if(VTK_LIBRARIES)
  target_link_libraries(ReadOBJ ${VTK_LIBRARIES})
else()
  target_link_libraries(ReadOBJ vtkHybrid )
endif()


cmake_minimum_required(VERSION 2.6)
 
PROJECT(SelectLines)
 
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})
 
ADD_EXECUTABLE(SelectLines SelectLines.cxx)
TARGET_LINK_LIBRARIES(SelectLines vtkHybrid)

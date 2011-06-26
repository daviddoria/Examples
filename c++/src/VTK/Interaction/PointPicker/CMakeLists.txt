cmake_minimum_required(VERSION 2.6)
 
PROJECT(PointPicker)
 
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})
 
ADD_EXECUTABLE(PointPicker PointPicker.cxx)
TARGET_LINK_LIBRARIES(PointPicker vtkHybrid)

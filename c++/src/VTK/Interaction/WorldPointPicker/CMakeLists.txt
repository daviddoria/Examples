cmake_minimum_required(VERSION 2.6)
 
PROJECT(WorldPointPicker)
 
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})
 
ADD_EXECUTABLE(WorldPointPicker WorldPointPicker.cxx)
TARGET_LINK_LIBRARIES(WorldPointPicker vtkHybrid)

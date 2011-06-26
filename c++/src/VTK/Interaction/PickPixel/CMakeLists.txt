cmake_minimum_required(VERSION 2.6)
 
PROJECT(PickPixel)
 
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})
 
ADD_EXECUTABLE(PickPixel PickPixel.cxx)
TARGET_LINK_LIBRARIES(PickPixel vtkHybrid)
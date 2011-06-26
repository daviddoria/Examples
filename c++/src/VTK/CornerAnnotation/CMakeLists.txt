cmake_minimum_required(VERSION 2.6)
 
PROJECT(CornerAnnotation)
 
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})
 
ADD_EXECUTABLE(CornerAnnotation CornerAnnotation.cxx)
TARGET_LINK_LIBRARIES(CornerAnnotation vtkHybrid)

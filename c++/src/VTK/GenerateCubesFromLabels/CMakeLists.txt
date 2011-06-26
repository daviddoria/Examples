cmake_minimum_required(VERSION 2.6)
 
PROJECT(GenerateCubesFromLabels)
 
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})
 
ADD_EXECUTABLE(GenerateCubesFromLabels GenerateCubesFromLabels.cxx)
 
TARGET_LINK_LIBRARIES(GenerateCubesFromLabels vtkHybrid)

cmake_minimum_required(VERSION 2.6)
 
PROJECT(FixedPointRayCastImage)
 
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})
 
ADD_EXECUTABLE(FixedPointRayCastImage FixedPointRayCastImage.cxx)
 
TARGET_LINK_LIBRARIES(FixedPointRayCastImage vtkHybrid vtkVolumeRendering)

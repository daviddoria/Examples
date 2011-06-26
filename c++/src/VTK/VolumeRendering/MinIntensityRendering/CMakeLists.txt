cmake_minimum_required(VERSION 2.6)
 
PROJECT(MinIntensityRendering)
 
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})
 
ADD_EXECUTABLE(MinIntensityRendering MinIntensityRendering.cxx)
 
TARGET_LINK_LIBRARIES(MinIntensityRendering vtkHybrid vtkVolumeRendering)

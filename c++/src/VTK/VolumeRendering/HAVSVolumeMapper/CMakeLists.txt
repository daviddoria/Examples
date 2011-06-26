cmake_minimum_required(VERSION 2.6)
 
PROJECT(HAVSVolumeMapper)
 
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})
 
ADD_EXECUTABLE(HAVSVolumeMapper HAVSVolumeMapper.cxx)
 
TARGET_LINK_LIBRARIES(HAVSVolumeMapper vtkHybrid vtkVolumeRendering)

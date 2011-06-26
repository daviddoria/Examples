cmake_minimum_required(VERSION 2.6)
 
PROJECT(SmartVolumeMapper)
 
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})
 
ADD_EXECUTABLE(SmartVolumeMapper SmartVolumeMapper.cxx)
 
TARGET_LINK_LIBRARIES(SmartVolumeMapper vtkHybrid vtkVolumeRendering)

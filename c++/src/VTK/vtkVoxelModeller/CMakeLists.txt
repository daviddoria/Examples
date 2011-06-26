cmake_minimum_required(VERSION 2.6)
PROJECT(VoxelModeller)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(VoxelModeller VoxelModeller.cxx)
TARGET_LINK_LIBRARIES(VoxelModeller vtkHybrid)

ADD_EXECUTABLE(Reader Reader.cxx)
TARGET_LINK_LIBRARIES(Reader vtkHybrid)


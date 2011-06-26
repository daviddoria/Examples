cmake_minimum_required(VERSION 2.6)
PROJECT(VoxelizePoints)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(VoxelizePoints VoxelizePoints.cxx)
TARGET_LINK_LIBRARIES(VoxelizePoints vtkHybrid)


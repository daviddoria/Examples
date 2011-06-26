cmake_minimum_required(VERSION 2.6)

PROJECT(LabelPlacementMapper)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(LabelPlacementMapper LabelPlacementMapper.cxx)
TARGET_LINK_LIBRARIES(LabelPlacementMapper vtkHybrid)

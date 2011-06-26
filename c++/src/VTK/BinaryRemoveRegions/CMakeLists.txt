cmake_minimum_required(VERSION 2.6)
PROJECT(BinaryRemoveRegions)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(BinaryRemoveRegions BinaryRemoveRegions.cxx vtkBinaryRemoveRegions.cxx)
TARGET_LINK_LIBRARIES(BinaryRemoveRegions vtkHybrid)


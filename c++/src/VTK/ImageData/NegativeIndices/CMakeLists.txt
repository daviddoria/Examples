cmake_minimum_required(VERSION 2.6)
PROJECT(NegativeIndices)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(NegativeIndices NegativeIndices.cxx)
TARGET_LINK_LIBRARIES(NegativeIndices vtkHybrid)


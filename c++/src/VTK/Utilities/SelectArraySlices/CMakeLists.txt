cmake_minimum_required(VERSION 2.6)
PROJECT(SelectArraySlices)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SelectArraySlices SelectArraySlices.cxx)
TARGET_LINK_LIBRARIES(SelectArraySlices vtkHybrid)


cmake_minimum_required(VERSION 2.6)
PROJECT(CreateDataSets)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(CreateDataSets CreateDataSets.cxx)
TARGET_LINK_LIBRARIES(CreateDataSets vtkHybrid)


cmake_minimum_required(VERSION 2.6)
PROJECT(Testing)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Testing Testing.cxx)
TARGET_LINK_LIBRARIES(Testing vtkHybrid)


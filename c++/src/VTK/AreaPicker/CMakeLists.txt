cmake_minimum_required(VERSION 2.6)

PROJECT(AreaPicker)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(AreaPicker AreaPicker.cxx)
TARGET_LINK_LIBRARIES(AreaPicker vtkHybrid)

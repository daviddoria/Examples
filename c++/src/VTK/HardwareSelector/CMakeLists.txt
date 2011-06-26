cmake_minimum_required(VERSION 2.6)

PROJECT(HardwareSelector)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(HardwareSelector HardwareSelector.cxx)

TARGET_LINK_LIBRARIES(HardwareSelector vtkHybrid)

cmake_minimum_required(VERSION 2.6)

PROJECT(StyleSwitch)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(StyleSwitch StyleSwitch.cxx)
TARGET_LINK_LIBRARIES(StyleSwitch vtkHybrid)

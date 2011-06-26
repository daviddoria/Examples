cmake_minimum_required(VERSION 2.6)

PROJECT(OverrideInteraction)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(OverrideInteraction OverrideInteraction.cpp)

TARGET_LINK_LIBRARIES(OverrideInteraction vtkHybrid)

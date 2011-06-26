cmake_minimum_required(VERSION 2.6)

PROJECT(ExitInteraction)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ExitInteraction ExitInteraction.cxx)
TARGET_LINK_LIBRARIES(ExitInteraction vtkHybrid)

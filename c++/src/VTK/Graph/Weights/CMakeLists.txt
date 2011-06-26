cmake_minimum_required(VERSION 2.6)

PROJECT(Weights)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Weights Weights.cxx)
TARGET_LINK_LIBRARIES(Weights vtkHybrid vtkInfovis)


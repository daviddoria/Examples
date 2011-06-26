cmake_minimum_required(VERSION 2.6)

PROJECT(PowerWeighting)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(PowerWeighting PowerWeighting.cxx)
TARGET_LINK_LIBRARIES(PowerWeighting vtkHybrid vtkTextAnalysis)

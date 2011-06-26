cmake_minimum_required(VERSION 2.6)

PROJECT(PCAAnalysisFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(PCAAnalysisFilter PCAAnalysisFilter.cxx)
TARGET_LINK_LIBRARIES(PCAAnalysisFilter vtkHybrid vtkInfovis)

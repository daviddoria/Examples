cmake_minimum_required(VERSION 2.6)

PROJECT(TextExtraction)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(TextExtraction TextExtraction.cxx)
TARGET_LINK_LIBRARIES(TextExtraction vtkHybrid vtkTextAnalysis)

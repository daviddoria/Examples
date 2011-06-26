cmake_minimum_required(VERSION 2.6)

PROJECT(VisualizeImageData)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(VisualizeImageData VisualizeImageData.cxx)
TARGET_LINK_LIBRARIES(VisualizeImageData vtkHybrid)


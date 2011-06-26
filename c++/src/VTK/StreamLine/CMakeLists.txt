cmake_minimum_required(VERSION 2.6)
PROJECT(StreamLine)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(StreamLine StreamLine.cxx)
TARGET_LINK_LIBRARIES(StreamLine vtkHybrid vtkWidgets)


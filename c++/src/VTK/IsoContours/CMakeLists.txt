cmake_minimum_required(VERSION 2.6)

PROJECT(IsoContours)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(IsoContours IsoContours.cxx)
TARGET_LINK_LIBRARIES(IsoContours vtkHybrid vtkWidgets)

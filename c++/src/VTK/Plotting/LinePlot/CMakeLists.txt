cmake_minimum_required(VERSION 2.6)

PROJECT(LinePlot)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(LinePlot LinePlot.cxx)

TARGET_LINK_LIBRARIES(LinePlot vtkHybrid vtkCharts)

cmake_minimum_required(VERSION 2.6)

PROJECT(KdTreePointLocatorVisualization)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(KdTreePointLocatorVisualization KdTreePointLocatorVisualization.cxx)
TARGET_LINK_LIBRARIES(KdTreePointLocatorVisualization vtkHybrid vtkWidgets)


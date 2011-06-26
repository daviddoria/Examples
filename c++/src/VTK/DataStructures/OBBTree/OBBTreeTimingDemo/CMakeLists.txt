cmake_minimum_required(VERSION 2.6)

PROJECT(OBBTreeTimingDemo)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(OBBTreeTimingDemo OBBTreeTimingDemo.cxx)
TARGET_LINK_LIBRARIES(OBBTreeTimingDemo vtkHybrid vtkCharts)


cmake_minimum_required(VERSION 2.6)

PROJECT(OctreeTimingDemo)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(OctreeTimingDemo OctreeTimingDemo.cxx)
TARGET_LINK_LIBRARIES(OctreeTimingDemo vtkHybrid vtkCharts)


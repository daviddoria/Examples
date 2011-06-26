cmake_minimum_required(VERSION 2.6)

PROJECT(OctreePointLocatorVisualizeLevels)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(OctreePointLocatorVisualizeLevels OctreePointLocatorVisualizeLevels.cxx)
TARGET_LINK_LIBRARIES(OctreePointLocatorVisualizeLevels vtkHybrid vtkWidgets)


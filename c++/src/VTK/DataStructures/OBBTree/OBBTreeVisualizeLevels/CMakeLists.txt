cmake_minimum_required(VERSION 2.6)

PROJECT(OBBTreeVisualizeLevels)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(OBBTreeVisualizeLevels OBBTreeVisualizeLevels.cxx)
TARGET_LINK_LIBRARIES(OBBTreeVisualizeLevels vtkHybrid vtkWidgets)


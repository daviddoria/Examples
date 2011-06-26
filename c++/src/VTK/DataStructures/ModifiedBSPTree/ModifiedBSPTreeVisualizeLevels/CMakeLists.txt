cmake_minimum_required(VERSION 2.6)

PROJECT(ModifiedBSPTreeVisualizeLevels)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ModifiedBSPTreeVisualizeLevels ModifiedBSPTreeVisualizeLevels.cxx)
TARGET_LINK_LIBRARIES(ModifiedBSPTreeVisualizeLevels vtkHybrid vtkWidgets)


cmake_minimum_required(VERSION 2.6)

PROJECT(Drawing)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Drawing Drawing.cxx)
TARGET_LINK_LIBRARIES(Drawing vtkHybrid)


ADD_EXECUTABLE(DrawingShapes DrawingShapes.cxx)
TARGET_LINK_LIBRARIES(DrawingShapes vtkHybrid)


ADD_EXECUTABLE(MultipleLines MultipleLines.cxx)
TARGET_LINK_LIBRARIES(MultipleLines vtkHybrid)



cmake_minimum_required(VERSION 2.6)

PROJECT(ArcParallelEdgeStrategy)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ArcParallelEdgeStrategy ArcParallelEdgeStrategy.cxx)
TARGET_LINK_LIBRARIES(ArcParallelEdgeStrategy vtkHybrid  vtkViews)


cmake_minimum_required(VERSION 2.6)

PROJECT(ColorVertexLabels)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ColorVertexLabels ColorVertexLabels.cxx)
TARGET_LINK_LIBRARIES(ColorVertexLabels vtkHybrid vtkInfovis vtkViews)

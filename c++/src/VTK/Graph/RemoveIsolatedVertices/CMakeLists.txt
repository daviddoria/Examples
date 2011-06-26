cmake_minimum_required(VERSION 2.6)

PROJECT(RemoveIsolatedVertices)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(RemoveIsolatedVertices RemoveIsolatedVertices.cxx)
TARGET_LINK_LIBRARIES(RemoveIsolatedVertices vtkHybrid vtkInfovis)


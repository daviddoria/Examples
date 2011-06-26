cmake_minimum_required(VERSION 2.6)

PROJECT(ColorVerticesLookupTable)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ColorVerticesLookupTable ColorVerticesLookupTable.cxx)
TARGET_LINK_LIBRARIES(ColorVerticesLookupTable vtkHybrid vtkInfovis vtkViews)


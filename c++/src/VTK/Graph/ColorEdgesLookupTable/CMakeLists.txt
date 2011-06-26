cmake_minimum_required(VERSION 2.6)

PROJECT(ColorEdgesLookupTable)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ColorEdgesLookupTable ColorEdgesLookupTable.cxx)
TARGET_LINK_LIBRARIES(ColorEdgesLookupTable vtkHybrid vtkInfovis vtkViews)


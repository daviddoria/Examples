cmake_minimum_required(VERSION 2.6)

PROJECT(GraphToPolydata)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(GraphToPolydata GraphToPolydata.cxx)
TARGET_LINK_LIBRARIES(GraphToPolydata vtkHybrid vtkInfovis)

ADD_EXECUTABLE(Demo Demo.cxx)
TARGET_LINK_LIBRARIES(Demo vtkHybrid vtkInfovis)


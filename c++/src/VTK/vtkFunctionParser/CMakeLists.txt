cmake_minimum_required(VERSION 2.6)

PROJECT(FunctionParser)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(FunctionParser FunctionParser.cxx)
TARGET_LINK_LIBRARIES(FunctionParser vtkHybrid)

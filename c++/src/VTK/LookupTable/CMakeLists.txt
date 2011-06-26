cmake_minimum_required(VERSION 2.6)

PROJECT(LookupTable)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(LookupTable LookupTable.cpp)
TARGET_LINK_LIBRARIES(LookupTable vtkHybrid vtkInfovis)

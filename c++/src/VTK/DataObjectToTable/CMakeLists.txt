cmake_minimum_required(VERSION 2.6)

PROJECT(DataObjectToTable)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(DataObjectToTable DataObjectToTable.cxx)

TARGET_LINK_LIBRARIES(DataObjectToTable vtkHybrid vtkInfovis)

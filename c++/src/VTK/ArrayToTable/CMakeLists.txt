cmake_minimum_required(VERSION 2.6)
PROJECT(ArrayToTable)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ArrayToTable ArrayToTable.cxx)
TARGET_LINK_LIBRARIES(ArrayToTable vtkHybrid vtkInfovis)


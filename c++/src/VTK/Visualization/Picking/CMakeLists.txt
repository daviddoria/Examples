cmake_minimum_required(VERSION 2.6)

PROJECT(Picking)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Picking Picking.cxx)
TARGET_LINK_LIBRARIES(Picking vtkHybrid vtkInfovis)

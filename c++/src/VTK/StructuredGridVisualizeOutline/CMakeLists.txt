cmake_minimum_required(VERSION 2.6)
PROJECT(StructuredGridVisualizeOutline)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(StructuredGridVisualizeOutline StructuredGridVisualizeOutline.cxx)
TARGET_LINK_LIBRARIES(StructuredGridVisualizeOutline vtkHybrid)

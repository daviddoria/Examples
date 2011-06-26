cmake_minimum_required(VERSION 2.6)
PROJECT(ApplyColors)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ApplyColors ApplyColors.cxx)
TARGET_LINK_LIBRARIES(ApplyColors vtkHybrid vtkInfovis)


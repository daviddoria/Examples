cmake_minimum_required(VERSION 2.6)

PROJECT(ColorTransferFunction)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ColorTransferFunction ColorTransferFunction.cxx)
TARGET_LINK_LIBRARIES(ColorTransferFunction vtkHybrid vtkInfovis)

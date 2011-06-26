cmake_minimum_required(VERSION 2.6)

PROJECT(DiscretizableColorTransferFunction)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(DiscretizableColorTransferFunction DiscretizableColorTransferFunction.cxx)
TARGET_LINK_LIBRARIES(DiscretizableColorTransferFunction vtkHybrid vtkInfovis)

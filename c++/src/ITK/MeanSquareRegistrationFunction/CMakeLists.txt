cmake_minimum_required(VERSION 2.6)

PROJECT(MeanSquareRegistrationFunction)

include_directories(/home/doriad/src/ITK/Wrapping/WrapITK/ExternalProjects/ItkVtkGlue/src/)
#include_directories(${ITK_SOURCE_DIR}/Wrapping/WrapITK/ExternalProjects/ItkVtkGlue/src/)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(MeanSquareRegistrationFunction MeanSquareRegistrationFunction.cxx)
TARGET_LINK_LIBRARIES(MeanSquareRegistrationFunction
vtkHybrid
ITKBasicFilters ITKCommon ITKIO)

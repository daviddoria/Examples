cmake_minimum_required(VERSION 2.6)

PROJECT(RelabelComponentImageFilter)

include_directories(/home/doriad/src/ITK/Wrapping/WrapITK/ExternalProjects/ItkVtkGlue/src/)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(RelabelComponentImageFilter RelabelComponentImageFilter.cxx)
TARGET_LINK_LIBRARIES(RelabelComponentImageFilter vtkHybrid
ITKIO ITKNumerics ITKBasicFilters ITKCommon
)
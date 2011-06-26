cmake_minimum_required(VERSION 2.6)

PROJECT(AntiAliasBinaryImageFilter)

include_directories(/home/doriad/src/ITK/Wrapping/WrapITK/ExternalProjects/ItkVtkGlue/src/)
include_directories(/home/doriad/itkwikiexamples/ItkVtkGlue)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(AntiAliasBinaryImageFilter AntiAliasBinaryImageFilter.cxx /home/doriad/itkwikiexamples/ItkVtkGlue/QuickView.cxx)
TARGET_LINK_LIBRARIES(AntiAliasBinaryImageFilter vtkHybrid ITKBasicFilters ITKIO)

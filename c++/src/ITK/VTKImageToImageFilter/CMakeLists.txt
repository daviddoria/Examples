cmake_minimum_required(VERSION 2.6)

PROJECT(VTKImageToImageFilterDemo)

include_directories(/home/doriad/ITKWikiExamples/ItkVtkGlue)
include_directories(/home/doriad/src/ITK/Wrapping/WrapITK/ExternalProjects/ItkVtkGlue/src/)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(VTKImageToImageFilterDemo VTKImageToImageFilterDemo.cxx /home/doriad/ITKWikiExamples/ItkVtkGlue/QuickView.cxx)
TARGET_LINK_LIBRARIES(VTKImageToImageFilterDemo
vtkHybrid
ITKBasicFilters ITKCommon ITKIO
)
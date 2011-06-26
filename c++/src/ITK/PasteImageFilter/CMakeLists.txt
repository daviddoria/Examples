cmake_minimum_required(VERSION 2.6)

PROJECT(PasteImageFilter)

include_directories(/home/doriad/src/ITK/Wrapping/WrapITK/ExternalProjects/ItkVtkGlue/src/)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(PasteImageFilter PasteImageFilter.cxx)
TARGET_LINK_LIBRARIES(PasteImageFilter
vtkHybrid
ITKBasicFilters ITKCommon ITKIO)
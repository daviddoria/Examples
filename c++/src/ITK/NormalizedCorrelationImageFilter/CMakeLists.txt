cmake_minimum_required(VERSION 2.6)

PROJECT(NormalizedCorrelationImageFilter)

include_directories(/home/doriad/ITKWikiExamples/ItkVtkGlue)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(NormalizedCorrelationImageFilter NormalizedCorrelationImageFilter.cpp /home/doriad/ITKWikiExamples/ItkVtkGlue/QuickView.cxx)
TARGET_LINK_LIBRARIES(NormalizedCorrelationImageFilter
vtkHybrid
ITKBasicFilters ITKCommon ITKIO)

ADD_EXECUTABLE(Color Color.cpp /home/doriad/ITKWikiExamples/ItkVtkGlue/QuickView.cxx)
TARGET_LINK_LIBRARIES(Color
vtkHybrid
ITKBasicFilters ITKCommon ITKIO)

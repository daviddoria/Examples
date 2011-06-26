cmake_minimum_required(VERSION 2.6)

PROJECT(TranslationTransform)

include_directories(/home/doriad/ITKWikiExamples/ItkVtkGlue)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(TranslationTransform TranslationTransform.cxx /home/doriad/ITKWikiExamples/ItkVtkGlue/QuickView.cxx)
TARGET_LINK_LIBRARIES(TranslationTransform
vtkHybrid
ITKBasicFilters ITKCommon ITKIO)

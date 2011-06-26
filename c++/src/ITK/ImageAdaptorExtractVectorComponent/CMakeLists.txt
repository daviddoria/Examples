cmake_minimum_required(VERSION 2.6)

PROJECT(ImageAdaptorExtractVectorComponent)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ImageAdaptorExtractVectorComponent ImageAdaptorExtractVectorComponent.cxx)
TARGET_LINK_LIBRARIES(ImageAdaptorExtractVectorComponent ITKIO)

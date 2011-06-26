cmake_minimum_required(VERSION 2.6)

PROJECT(KMeansClassification)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(KMeansClassification KMeansClassification.cxx)
TARGET_LINK_LIBRARIES(KMeansClassification ITKNumerics ITKIO)

ADD_EXECUTABLE(original original.cxx)
TARGET_LINK_LIBRARIES(original ITKNumerics ITKIO)


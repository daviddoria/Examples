cmake_minimum_required(VERSION 2.6)

PROJECT(SampleMeanShiftClustering)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(SampleMeanShiftClustering SampleMeanShiftClustering.cxx)
TARGET_LINK_LIBRARIES(SampleMeanShiftClustering ITKCommon ITKNumerics ITKStatistics ITKIO)

ADD_EXECUTABLE(original original.cxx)
TARGET_LINK_LIBRARIES(original ITKCommon ITKNumerics ITKStatistics ITKIO)


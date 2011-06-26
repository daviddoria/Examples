cmake_minimum_required(VERSION 2.6)

PROJECT(ImageKmeansModelEstimator)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ImageKmeansModelEstimator ImageKmeansModelEstimator.cxx)
TARGET_LINK_LIBRARIES(ImageKmeansModelEstimator ITKIO ITKStatistics
vtkHybrid)

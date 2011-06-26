cmake_minimum_required(VERSION 2.6)

PROJECT(KdTreeBasedKMeansClustering)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(KdTreeBasedKMeansClustering_1D KdTreeBasedKMeansClustering_1D.cxx)
TARGET_LINK_LIBRARIES(KdTreeBasedKMeansClustering_1D
ITKBasicFilters ITKCommon ITKIO ITKStatistics
vtkHybrid)

ADD_EXECUTABLE(KdTreeBasedKMeansClustering_2D KdTreeBasedKMeansClustering_2D.cxx)
TARGET_LINK_LIBRARIES(KdTreeBasedKMeansClustering_2D
ITKBasicFilters ITKCommon ITKIO ITKStatistics
vtkHybrid)

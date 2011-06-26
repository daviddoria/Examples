cmake_minimum_required(VERSION 2.6)

PROJECT(VectorRescaleIntensityImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(VectorRescaleIntensityImageFilter VectorRescaleIntensityImageFilter.cxx)
TARGET_LINK_LIBRARIES(VectorRescaleIntensityImageFilter ${ITK_LIBRARIES})

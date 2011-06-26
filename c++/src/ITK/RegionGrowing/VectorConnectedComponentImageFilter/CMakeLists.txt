cmake_minimum_required(VERSION 2.6)

PROJECT(VectorConnectedComponentImageFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(VectorConnectedComponentImageFilter VectorConnectedComponentImageFilter.cxx)
TARGET_LINK_LIBRARIES(VectorConnectedComponentImageFilter ${ITK_LIBRARIES})


cmake_minimum_required(VERSION 2.6)

PROJECT(ImageToVectorImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ImageToVectorImageFilter ImageToVectorImageFilter.cxx)
TARGET_LINK_LIBRARIES(ImageToVectorImageFilter ${ITK_LIBRARIES})


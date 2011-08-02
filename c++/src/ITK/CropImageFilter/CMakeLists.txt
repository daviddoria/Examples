cmake_minimum_required(VERSION 2.6)

PROJECT(CropImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(CropImageFilter CropImageFilter.cxx)
TARGET_LINK_LIBRARIES(CropImageFilter ${ITK_LIBRARIES})

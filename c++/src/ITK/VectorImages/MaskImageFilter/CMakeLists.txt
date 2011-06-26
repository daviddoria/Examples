cmake_minimum_required(VERSION 2.6)

PROJECT(MaskVectorImage)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(MaskVectorImage MaskVectorImage.cxx)
TARGET_LINK_LIBRARIES(MaskVectorImage ${ITK_LIBRARIES})


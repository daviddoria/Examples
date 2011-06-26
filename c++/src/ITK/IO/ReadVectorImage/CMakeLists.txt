cmake_minimum_required(VERSION 2.6)

PROJECT(ReadVectorImage)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ReadVectorImage ReadVectorImage.cxx)
TARGET_LINK_LIBRARIES(ReadVectorImage ${ITK_LIBRARIES})

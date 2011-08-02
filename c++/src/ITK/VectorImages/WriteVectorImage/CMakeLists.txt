cmake_minimum_required(VERSION 2.6)

PROJECT(WriteVectorImage)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(WriteVectorImage WriteVectorImage.cxx)
TARGET_LINK_LIBRARIES(WriteVectorImage ${ITK_LIBRARIES})

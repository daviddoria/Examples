cmake_minimum_required(VERSION 2.6)

PROJECT(VectorContainer)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(VectorContainer VectorContainer.cxx)
TARGET_LINK_LIBRARIES(VectorContainer ITKCommon)


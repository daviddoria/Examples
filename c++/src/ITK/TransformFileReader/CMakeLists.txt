cmake_minimum_required(VERSION 2.6)

PROJECT(TransformFileReader)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(TransformFileReader TransformFileReader.cxx)
TARGET_LINK_LIBRARIES(TransformFileReader ${ITK_LIBRARIES})

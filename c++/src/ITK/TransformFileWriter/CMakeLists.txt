cmake_minimum_required(VERSION 2.6)

PROJECT(TransformFileWriter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(TransformFileWriter TransformFileWriter.cxx)
TARGET_LINK_LIBRARIES(TransformFileWriter ${ITK_LIBRARIES})


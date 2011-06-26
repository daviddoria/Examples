cmake_minimum_required(VERSION 2.6)

PROJECT(ImageFileWriter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ImageFileWriter ImageFileWriter.cxx)
TARGET_LINK_LIBRARIES(ImageFileWriter ${ITK_LIBRARIES})
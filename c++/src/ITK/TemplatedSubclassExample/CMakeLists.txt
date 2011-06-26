cmake_minimum_required(VERSION 2.6)

PROJECT(TemplatedSubclassExample)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(TemplatedSubclassExample TemplatedSubclassExample.cpp)
TARGET_LINK_LIBRARIES(TemplatedSubclassExample ITKIO)

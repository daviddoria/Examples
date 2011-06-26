cmake_minimum_required(VERSION 2.6)

PROJECT(BresenhamLine)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(BresenhamLine BresenhamLine.cxx)
TARGET_LINK_LIBRARIES(BresenhamLine ITKIO)

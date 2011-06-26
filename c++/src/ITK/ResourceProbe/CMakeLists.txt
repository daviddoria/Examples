cmake_minimum_required(VERSION 2.6)

PROJECT(ResourceProbe)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ResourceProbe ResourceProbe.cxx)
TARGET_LINK_LIBRARIES(ResourceProbe ITKIO)

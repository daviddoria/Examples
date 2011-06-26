cmake_minimum_required(VERSION 2.6)

PROJECT(VariableLengthVector)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(VariableLengthVector VariableLengthVector.cxx)
TARGET_LINK_LIBRARIES(VariableLengthVector ${ITK_LIBRARIES})


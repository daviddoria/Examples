cmake_minimum_required(VERSION 2.6)

PROJECT(BackwardDifferenceOperator)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(BackwardDifferenceOperator BackwardDifferenceOperator.cxx)
TARGET_LINK_LIBRARIES(BackwardDifferenceOperator ${ITK_LIBRARIES})

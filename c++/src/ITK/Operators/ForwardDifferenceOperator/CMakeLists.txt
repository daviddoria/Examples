cmake_minimum_required(VERSION 2.6)

PROJECT(ForwardDifferenceOperator)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ForwardDifferenceOperator ForwardDifferenceOperator.cxx)
TARGET_LINK_LIBRARIES(ForwardDifferenceOperator ${ITK_LIBRARIES})

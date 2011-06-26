cmake_minimum_required(VERSION 2.6)

PROJECT(DerivativeOperator)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(DerivativeOperator DerivativeOperator.cpp)
TARGET_LINK_LIBRARIES(DerivativeOperator ${ITK_LIBRARIES})

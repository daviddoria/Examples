cmake_minimum_required(VERSION 2.6)

PROJECT(SobelOperator)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(SobelOperator SobelOperator.cpp)
TARGET_LINK_LIBRARIES(SobelOperator ${ITK_LIBRARIES})


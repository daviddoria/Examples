cmake_minimum_required(VERSION 2.6)

PROJECT(LaplacianOperator)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(LaplacianOperator LaplacianOperator.cxx)
TARGET_LINK_LIBRARIES(LaplacianOperator ${ITK_LIBRARIES})

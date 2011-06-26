cmake_minimum_required(VERSION 2.6)

PROJECT(GaussianOperator)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(GaussianOperator GaussianOperator.cxx)
TARGET_LINK_LIBRARIES(GaussianOperator ${ITK_LIBRARIES})

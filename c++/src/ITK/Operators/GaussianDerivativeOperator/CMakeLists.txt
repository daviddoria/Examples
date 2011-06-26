cmake_minimum_required(VERSION 2.6)

PROJECT(GaussianDerivativeOperator)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(GaussianDerivativeOperator GaussianDerivativeOperator.cxx)
TARGET_LINK_LIBRARIES(GaussianDerivativeOperator ${ITK_LIBRARIES})

cmake_minimum_required(VERSION 2.6)
PROJECT(LUFactorization)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(LUFactorization LUFactorization.cxx)
TARGET_LINK_LIBRARIES(LUFactorization vtkHybrid)

cmake_minimum_required(VERSION 2.6)

PROJECT(SparseMatrixSolver)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})

#if you didn't 'make install' to a standard location, set EIGEN3_INCLUDE_DIR to /home/doriad/bin/eigen3/include/eigen3
FIND_PACKAGE(Eigen3 REQUIRED) #requires you to copy FindEigen3.cmake into the source directory
include_directories(${EIGEN3_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR}/unsupported/)

FIND_PACKAGE(Umfpack REQUIRED) #requires you to copy FindUmfpack.cmake into the source directory
FIND_PACKAGE(BLAS REQUIRED)

include_directories(${UMFPACK_INCLUDES})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET")

ADD_EXECUTABLE(SparseMatrixSolver SparseMatrixSolver.cpp )
target_link_libraries(SparseMatrixSolver umfpack)

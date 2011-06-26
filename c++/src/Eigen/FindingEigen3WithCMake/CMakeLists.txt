cmake_minimum_required(VERSION 2.6)

PROJECT(SparseMatrixSolver)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})

FIND_PACKAGE(Eigen2 REQUIRED) #requires you to copy FindEigen.cmake into the source directory
include_directories(${EIGEN2_INCLUDE_DIR})

FIND_PACKAGE(Umfpack REQUIRED) #requires you to copy FindUmfpack.cmake into the source directory
FIND_PACKAGE(BLAS REQUIRED)

include_directories(${UMFPACK_INCLUDES})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET")

ADD_EXECUTABLE(SparseMatrixSolver SparseMatrixSolver.cpp )
target_link_libraries(SparseMatrixSolver umfpack)

ADD_EXECUTABLE(SparseLeastSquares SparseLeastSquares.cpp )
target_link_libraries(SparseLeastSquares umfpack)

#ADD_EXECUTABLE(BuiltIn BuiltIn.cpp )
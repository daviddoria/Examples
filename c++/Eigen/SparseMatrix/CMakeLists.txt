cmake_minimum_required(VERSION 2.6)

PROJECT(SparseMatrix)
include_directories(/media/portable/src/Eigen)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET")
ADD_EXECUTABLE(SparseMatrix SparseMatrix.cpp )

cmake_minimum_required(VERSION 2.6)

Project(SparseMatrix)

FIND_PACKAGE(VXL REQUIRED)
INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)

ADD_EXECUTABLE(SparseMatrix SparseMatrix.cpp)
TARGET_LINK_LIBRARIES(SparseMatrix vnl)

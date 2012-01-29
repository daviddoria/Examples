cmake_minimum_required(VERSION 2.6)

Project(Eigen)

FIND_PACKAGE(VXL REQUIRED)
INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)

ADD_EXECUTABLE(Eigen Eigen.cpp)

TARGET_LINK_LIBRARIES(Eigen vnl vnl_algo)

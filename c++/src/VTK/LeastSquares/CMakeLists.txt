cmake_minimum_required(VERSION 2.6)
PROJECT(LeastSquares)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(LeastSquares LeastSquares.cxx)
TARGET_LINK_LIBRARIES(LeastSquares vtkHybrid)

ADD_EXECUTABLE(HomogeneousLeastSquares HomogeneousLeastSquares.cxx)
TARGET_LINK_LIBRARIES(HomogeneousLeastSquares vtkHybrid)


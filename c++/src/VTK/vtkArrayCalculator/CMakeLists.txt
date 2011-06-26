cmake_minimum_required(VERSION 2.6)
PROJECT(ArrayCalculator)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ArrayCalculator ArrayCalculator.cxx)
TARGET_LINK_LIBRARIES(ArrayCalculator vtkHybrid)


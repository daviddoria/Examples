cmake_minimum_required(VERSION 2.6)

PROJECT(DenseArrayOfSmartPointers)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(DenseArrayOfSmartPointers DenseArrayOfSmartPointers.cxx vtkRay.cxx)
TARGET_LINK_LIBRARIES(DenseArrayOfSmartPointers vtkHybrid)

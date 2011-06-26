cmake_minimum_required(VERSION 2.6)

PROJECT(ForwardDeclareSmartPointer)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ForwardDeclareSmartPointer ForwardDeclareSmartPointer.cxx vtkRay.cxx)
TARGET_LINK_LIBRARIES(ForwardDeclareSmartPointer vtkHybrid)

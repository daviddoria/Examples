cmake_minimum_required(VERSION 2.6)

PROJECT(SmartPointerMember)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SmartPointerMember SmartPointerMember.cxx MyClass.cxx)
TARGET_LINK_LIBRARIES(SmartPointerMember vtkHybrid)

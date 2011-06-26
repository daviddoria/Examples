cmake_minimum_required(VERSION 2.6)

PROJECT(NonSmartPointerMember)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(NonSmartPointerMember NonSmartPointerMember.cxx MyClass.cxx)
TARGET_LINK_LIBRARIES(NonSmartPointerMember vtkHybrid)

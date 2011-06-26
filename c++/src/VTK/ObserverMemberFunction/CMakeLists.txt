cmake_minimum_required(VERSION 2.6)

PROJECT(ObserverMemberFunction)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ObserverMemberFunction ObserverMemberFunction.cxx)

TARGET_LINK_LIBRARIES(ObserverMemberFunction vtkHybrid)

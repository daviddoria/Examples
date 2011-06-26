cmake_minimum_required(VERSION 2.6)

PROJECT(InteractorStyleUser)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(InteractorStyleUser InteractorStyleUser.cxx)
TARGET_LINK_LIBRARIES(InteractorStyleUser vtkHybrid)

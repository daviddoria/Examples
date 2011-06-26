cmake_minimum_required(VERSION 2.6)

PROJECT(ShepardMethod)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ShepardMethod ShepardMethod.cxx)
TARGET_LINK_LIBRARIES(ShepardMethod vtkHybrid)

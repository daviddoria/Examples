cmake_minimum_required(VERSION 2.6)

PROJECT(RandomNumbers)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(RandomUniform RandomUniform.cxx)
TARGET_LINK_LIBRARIES(RandomUniform vtkHybrid)

ADD_EXECUTABLE(RandomGaussian RandomGaussian.cxx)
TARGET_LINK_LIBRARIES(RandomGaussian vtkHybrid)

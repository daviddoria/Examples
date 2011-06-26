cmake_minimum_required(VERSION 2.6)
PROJECT(ProgrammableFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ProgrammableFilter ProgrammableFilter.cxx)
TARGET_LINK_LIBRARIES(ProgrammableFilter vtkHybrid)


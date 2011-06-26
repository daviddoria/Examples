cmake_minimum_required(VERSION 2.6)

PROJECT(CreateDatabase)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(CreateDatabase CreateDatabase.cxx)
TARGET_LINK_LIBRARIES(CreateDatabase vtkHybrid)

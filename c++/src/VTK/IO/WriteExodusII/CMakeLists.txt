cmake_minimum_required(VERSION 2.6)

PROJECT(WriteExodusII)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(WriteExodusII WriteExodusII.cxx)
TARGET_LINK_LIBRARIES(WriteExodusII vtkHybrid)


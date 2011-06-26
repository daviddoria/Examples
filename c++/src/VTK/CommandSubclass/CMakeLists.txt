cmake_minimum_required(VERSION 2.6)

PROJECT(CommandSubclass)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(CommandSubclass CommandSubclass.cxx)
TARGET_LINK_LIBRARIES(CommandSubclass vtkHybrid)

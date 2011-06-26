cmake_minimum_required(VERSION 2.6)

PROJECT(StaticImage)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(StaticImage StaticImage.cxx)
TARGET_LINK_LIBRARIES(StaticImage vtkHybrid)


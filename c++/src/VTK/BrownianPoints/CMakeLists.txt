cmake_minimum_required(VERSION 2.6)

PROJECT(BrownianPoints)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(BrownianPoints BrownianPoints.cxx)

TARGET_LINK_LIBRARIES(BrownianPoints vtkHybrid)

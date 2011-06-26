cmake_minimum_required(VERSION 2.6)

PROJECT(BoostKruskalMinimumSpanningTree)

#FIND_PACKAGE(Boost 1.38 COMPONENTS program_options required)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(BoostKruskalMinimumSpanningTree BoostKruskalMinimumSpanningTree.cxx)
TARGET_LINK_LIBRARIES(BoostKruskalMinimumSpanningTree vtkHybrid vtkInfovis)

ADD_EXECUTABLE(Demo Demo.cxx)
TARGET_LINK_LIBRARIES(Demo vtkHybrid vtkInfovis)

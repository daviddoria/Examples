cmake_minimum_required(VERSION 2.6)
PROJECT(PolyDataConnectivityFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(LargestRegion LargestRegion.cxx)
TARGET_LINK_LIBRARIES(LargestRegion vtkHybrid)

ADD_EXECUTABLE(SpecifiedRegion SpecifiedRegion.cxx)
TARGET_LINK_LIBRARIES(SpecifiedRegion vtkHybrid)

ADD_EXECUTABLE(AllRegions AllRegions.cxx)
TARGET_LINK_LIBRARIES(AllRegions vtkHybrid)


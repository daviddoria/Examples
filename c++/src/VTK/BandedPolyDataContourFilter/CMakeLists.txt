cmake_minimum_required(VERSION 2.6)
 
PROJECT(BandedPolyDataContourFilter)
 
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})
 
ADD_EXECUTABLE(BandedPolyDataContourFilter BandedPolyDataContourFilter.cxx)
 
TARGET_LINK_LIBRARIES(BandedPolyDataContourFilter vtkHybrid)

cmake_minimum_required(VERSION 2.6)
PROJECT(ExtractSelectedThresholds)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ExtractSelectedThresholds ExtractSelectedThresholds.cxx)
TARGET_LINK_LIBRARIES(ExtractSelectedThresholds vtkHybrid)


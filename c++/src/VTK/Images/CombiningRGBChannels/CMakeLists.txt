cmake_minimum_required(VERSION 2.6)

PROJECT(CombiningRGBChannels)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(CombiningRGBChannels CombiningRGBChannels.cxx)
TARGET_LINK_LIBRARIES(CombiningRGBChannels vtkHybrid)

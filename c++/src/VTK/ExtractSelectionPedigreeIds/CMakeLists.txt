cmake_minimum_required(VERSION 2.6)
PROJECT(ExtractSelectionPedigreeIds)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ExtractSelectionPedigreeIds ExtractSelectionPedigreeIds.cxx)
TARGET_LINK_LIBRARIES(ExtractSelectionPedigreeIds vtkHybrid)


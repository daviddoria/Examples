cmake_minimum_required(VERSION 2.6)
PROJECT(ExtractSelection)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ExtractSelection ExtractSelection.cxx)
TARGET_LINK_LIBRARIES(ExtractSelection vtkHybrid)

#ADD_EXECUTABLE(Test Test.cxx)
#TARGET_LINK_LIBRARIES(Test vtkHybrid)


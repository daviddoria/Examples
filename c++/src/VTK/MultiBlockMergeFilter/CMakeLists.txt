cmake_minimum_required(VERSION 2.6)

PROJECT(MultiBlockMergeFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(MultiBlockMergeFilter MultiBlockMergeFilter.cxx)
TARGET_LINK_LIBRARIES(MultiBlockMergeFilter ${VTK_LIBRARIES})

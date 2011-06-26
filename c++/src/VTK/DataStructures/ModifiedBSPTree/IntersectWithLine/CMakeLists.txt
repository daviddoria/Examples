cmake_minimum_required(VERSION 2.6)

PROJECT(ModifiedBSPTree_IntersectWithLine)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ModifiedBSPTree_IntersectWithLine ModifiedBSPTree_IntersectWithLine.cxx)
TARGET_LINK_LIBRARIES(ModifiedBSPTree_IntersectWithLine vtkHybrid)

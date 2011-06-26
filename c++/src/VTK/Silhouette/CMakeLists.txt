cmake_minimum_required(VERSION 2.6)
PROJECT(Silhouette)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Silhouette Silhouette.cxx)
TARGET_LINK_LIBRARIES(Silhouette vtkHybrid)


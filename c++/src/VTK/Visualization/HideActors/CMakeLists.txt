cmake_minimum_required(VERSION 2.6)

PROJECT(HideActors)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(HideActors HideActors.cxx)
TARGET_LINK_LIBRARIES(HideActors vtkHybrid)

ADD_EXECUTABLE(HideActor HideActor.cxx)
TARGET_LINK_LIBRARIES(HideActor vtkHybrid)

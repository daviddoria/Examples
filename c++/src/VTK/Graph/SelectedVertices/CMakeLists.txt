cmake_minimum_required(VERSION 2.6)

PROJECT(SelectedVertices)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SelectedVertices SelectedVertices.cxx)
TARGET_LINK_LIBRARIES(SelectedVertices vtkHybrid vtkInfovis vtkViews)

ADD_EXECUTABLE(SelectedVerticesObserver SelectedVerticesObserver.cxx)
TARGET_LINK_LIBRARIES(SelectedVerticesObserver vtkHybrid vtkInfovis vtkViews)

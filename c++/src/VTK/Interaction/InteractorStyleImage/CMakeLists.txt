cmake_minimum_required(VERSION 2.6)

PROJECT(InteractorStyleImage)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(InteractorStyleImage InteractorStyleImage.cxx)
TARGET_LINK_LIBRARIES(InteractorStyleImage vtkHybrid)


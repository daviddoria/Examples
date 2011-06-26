cmake_minimum_required(VERSION 2.6)

PROJECT(MutableGraphHelper)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(MutableGraphHelper MutableGraphHelper.cxx)
TARGET_LINK_LIBRARIES(MutableGraphHelper vtkHybrid vtkInfovis vtkViews)


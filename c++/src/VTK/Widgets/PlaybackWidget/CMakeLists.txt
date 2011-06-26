cmake_minimum_required(VERSION 2.6)

PROJECT(PlaybackWidget)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(PlaybackWidget PlaybackWidget.cxx)
TARGET_LINK_LIBRARIES(PlaybackWidget vtkHybrid vtkWidgets)

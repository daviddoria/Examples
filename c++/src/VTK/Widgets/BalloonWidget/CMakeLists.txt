cmake_minimum_required(VERSION 2.6)

PROJECT(BalloonWidget)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(BalloonWidget BalloonWidget.cxx)
TARGET_LINK_LIBRARIES(BalloonWidget vtkHybrid vtkWidgets)

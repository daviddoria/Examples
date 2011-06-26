cmake_minimum_required(VERSION 2.6)

PROJECT(Slider)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Slider Slider.cxx)
TARGET_LINK_LIBRARIES(Slider vtkHybrid vtkWidgets)

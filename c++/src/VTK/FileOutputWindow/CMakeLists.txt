cmake_minimum_required(VERSION 2.6)
PROJECT(FileOutputWindow)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(FileOutputWindow FileOutputWindow.cxx)
TARGET_LINK_LIBRARIES(FileOutputWindow vtkHybrid)


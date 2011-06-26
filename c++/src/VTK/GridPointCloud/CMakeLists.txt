cmake_minimum_required(VERSION 2.6)
PROJECT(GridPointCloud)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(GridPointCloud GridPointCloud.cxx)
TARGET_LINK_LIBRARIES(GridPointCloud vtkHybrid)

ADD_EXECUTABLE(TestFindCell TestFindCell.cxx)
TARGET_LINK_LIBRARIES(TestFindCell vtkHybrid)


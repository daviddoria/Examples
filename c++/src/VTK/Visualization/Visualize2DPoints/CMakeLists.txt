cmake_minimum_required(VERSION 2.6)

PROJECT(Visualization)

set(CMAKE_BUILD_TYPE Debug)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Points2D Points2D.cpp)
TARGET_LINK_LIBRARIES(Points2D vtkHybrid)

ADD_EXECUTABLE(VisualizeSphere VisualizeSphere.cpp)
TARGET_LINK_LIBRARIES(VisualizeSphere vtkHybrid)

ADD_EXECUTABLE(VisualizeVTPFile VisualizeVTPFile)
TARGET_LINK_LIBRARIES(VisualizeVTPFile vtkHybrid)

ADD_EXECUTABLE(SphereTrackball SphereTrackball.cpp)
TARGET_LINK_LIBRARIES(SphereTrackball vtkHybrid)

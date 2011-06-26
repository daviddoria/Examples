cmake_minimum_required(VERSION 2.6)

PROJECT(ShapedNeighborhoodIterator)

# include_directories(/home/doriad/src/ITK/Wrapping/WrapITK/ExternalProjects/ItkVtkGlue/src/)
# 
# FIND_PACKAGE(VTK REQUIRED)
# INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ShapedNeighborhoodIterator ShapedNeighborhoodIterator.cxx)
TARGET_LINK_LIBRARIES(ShapedNeighborhoodIterator ${ITK_LIBRARIES}) #vtkHybrid 
# 
# ADD_EXECUTABLE(ShapedNeighborhoodIterator_SmallRegion ShapedNeighborhoodIterator_SmallRegion.cxx)
# TARGET_LINK_LIBRARIES(ShapedNeighborhoodIterator_SmallRegion ${ITK_LIBRARIES}) #vtkHybrid 

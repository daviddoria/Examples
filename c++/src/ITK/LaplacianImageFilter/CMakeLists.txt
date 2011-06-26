cmake_minimum_required(VERSION 2.6)

PROJECT(LaplacianImageFilter)

include_directories(/home/doriad/src/ITK/Wrapping/WrapITK/ExternalProjects/ItkVtkGlue/src/)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

find_package(ItkVtkGlue REQUIRED)
include(${ItkVtkGlue_USE_FILE})

# ADD_EXECUTABLE(LaplacianImageFilterUnsignedChar LaplacianImageFilterUnsignedChar.cxx)
# TARGET_LINK_LIBRARIES(LaplacianImageFilterUnsignedChar ${ITK_LIBRARIES})

ADD_EXECUTABLE(LaplacianImageFilterFloat LaplacianImageFilterFloat.cxx)
TARGET_LINK_LIBRARIES(LaplacianImageFilterFloat ${ITK_LIBRARIES} ItkVtkGlue vtkHybrid)

ADD_EXECUTABLE(LaplacianImageFilterVectorImage LaplacianImageFilterVectorImage.cxx)
TARGET_LINK_LIBRARIES(LaplacianImageFilterVectorImage ${ITK_LIBRARIES} ItkVtkGlue vtkHybrid)

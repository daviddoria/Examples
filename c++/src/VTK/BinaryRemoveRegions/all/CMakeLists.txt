#root CMakeLists.txt
CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
IF(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
ENDIF(COMMAND CMAKE_POLICY)

# This project is designed to be built outside the Insight source tree.
# Must be defined BEFORE everything else
PROJECT(vtkFilterTest)

# Find VTK
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

#does all the standard necessary CMakeLists stuff for ITK/VTK usage
#include ITK, VTK, and setup my pv-plugin directories ...
#INCLUDE("/home/christian/programming/pv-plugins/conf.cmake")

ADD_EXECUTABLE(vtkFilterTest vtkFilterTest.cxx 
#      ${PV_PLUGIN_DIR}/Segmentation/BinaryRemoveRegions/vtkBinaryRemoveRegions.cxx
       vtkBinaryRemoveRegions.cxx
)

TARGET_LINK_LIBRARIES(vtkFilterTest
      vtkImaging vtkIO vtkCommon
)

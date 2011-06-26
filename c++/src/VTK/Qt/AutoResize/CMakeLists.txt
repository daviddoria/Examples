cmake_minimum_required(VERSION 2.8)
 
PROJECT(AutoResize)

FIND_PACKAGE(VTK)
INCLUDE(${VTK_USE_FILE}) # include UseVTK.cmake
 
FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})  # include UseQt4.cmake
 
# support for out-of-source build
INCLUDE_DIRECTORIES(
  ${CMAKE_CURRENT_BINARY_DIR} #this is where ui_SimpleViewUI.h is generated
  ${CMAKE_CURRENT_SOURCE_DIR}
)
 
# Set your files and resources here
SET(SimpleViewSrcs AutoResize.cxx Form.cxx)
SET(SimpleViewUI Form.ui)
SET(SimpleViewHeaders Form.h)
 
QT4_WRAP_UI(UISrcs ${SimpleViewUI})
QT4_WRAP_CPP(MOCSrcs ${SimpleViewHeaders} )

ADD_EXECUTABLE(AutoResize ${SimpleViewSrcs} ${UISrcs} ${MOCSrcs})
TARGET_LINK_LIBRARIES(AutoResize QVTK )

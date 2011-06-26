cmake_minimum_required(VERSION 2.9)

PROJECT(SideBySideRenderWindows)

FIND_PACKAGE(VTK)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

# Set your files and resources here
SET(SimpleViewSrcs SideBySideRenderWindows.cxx SimpleView.cxx)
SET(SimpleViewUI SimpleView.ui)
SET(SimpleViewHeaders SimpleView.h)

QT4_WRAP_UI(UISrcs ${SimpleViewUI})
QT4_WRAP_CPP(MOCSrcs ${SimpleViewHeaders} )

SOURCE_GROUP("Resources" FILES
  ${SimpleViewUI}
)

SOURCE_GROUP("Generated" FILES
  ${UISrcs}
  ${MOCSrcs}
  ${RCS_SOURCES}
)

ADD_EXECUTABLE( SideBySideRenderWindows ${SimpleViewSrcs} ${UISrcs} ${MOCSrcs})
TARGET_LINK_LIBRARIES( SideBySideRenderWindows QVTK )

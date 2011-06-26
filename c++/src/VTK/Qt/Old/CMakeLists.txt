PROJECT(RenderWindow)

FIND_PACKAGE(VTK)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

# Set your files and resources here
SET(SimpleViewSrcs RenderWindow.cxx SimpleView.cxx)
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
#  ${RCS_SOURCES}
)

ADD_EXECUTABLE( RenderWindow ${SimpleViewSrcs} ${UISrcs} ${MOCSrcs})
TARGET_LINK_LIBRARIES( RenderWindow QVTK )
PROJECT(QtBorderWidget)

FIND_PACKAGE(VTK)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})
#INCLUDE_DIRECTORIES(.)
INCLUDE_DIRECTORIES(/media/portable/Examples/c++/bin/VTK/Qt/BorderWidget)

SET(SimpleViewSrcs QtBorderWidget.cxx SimpleView.cxx)
SET(SimpleViewUI SimpleView.ui)
SET(SimpleViewHeaders SimpleView.h)

QT4_WRAP_UI(UISrcs ${SimpleViewUI})
QT4_WRAP_CPP(MOCSrcs ${SimpleViewHeaders} )

ADD_EXECUTABLE( QtBorderWidget ${SimpleViewSrcs} ${UISrcs} ${MOCSrcs})
TARGET_LINK_LIBRARIES( QtBorderWidget QVTK )

cmake_minimum_required(VERSION 2.6)

PROJECT(RenderWindowNoUiFile)

FIND_PACKAGE(VTK)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})


#QT4_WRAP_UI(UISrcs ${SimpleViewUI})
#QT4_WRAP_CPP(MOCSrcs ${SimpleViewHeaders} )
#QT4_ADD_RESOURCES(ResourceSrcs ${SimpleViewResources})

#SOURCE_GROUP("Resources" FILES
#  ${SimpleViewUI}
#  ${SimpleViewResources}
#  ${EXE_ICON}
#)

#SOURCE_GROUP("Generated" FILES
#  ${UISrcs}
#  ${MOCSrcs}
#  ${ResourceSrcs}
#  ${RCS_SOURCES}
#)

ADD_EXECUTABLE(RenderWindowNoUiFile RenderWindowNoUiFile.cxx)
TARGET_LINK_LIBRARIES( RenderWindowNoUiFile QVTK )

cmake_minimum_required(VERSION 2.6)

PROJECT(KFileWidgetDemo)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

find_package(KDE4 4.5.0 REQUIRED)
include_directories( ${KDE4_INCLUDES} )

QT4_WRAP_UI(UISrcs form.ui)
#QT4_WRAP_CPP(MOCSrcs form.h) #don't manual MOC of you are going to automoc (using kde4_add_executable)

include_directories(${include_directories} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

kde4_add_executable(KFileWidgetDemo main.cpp form.h form.cpp ${MOCSrcs} ${UISrcs})
TARGET_LINK_LIBRARIES(KFileWidgetDemo ${QT_LIBRARIES} ${KDE_LIBRARIES} ${KDE4_KDEUI_LIBS} ${KDE4_KFILE_LIBS})

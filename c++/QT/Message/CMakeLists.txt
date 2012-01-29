# need Ubuntu package qtmobility-dev (doesn't seem to have Messaging?)

# when trying to build qtmobility: fatal error: QGLWidget: No such file or directory
# fixed by installing libqt4-opengl-dev

# fatal error: QDBusContext: No such file or directory
cmake_minimum_required(VERSION 2.6)

PROJECT(Message)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" )

#test_find_package(QtMobility QTMOBILITY INCLUDE_DIR INCLUDE_DIRS LIBRARIES)


find_package(QtMobility REQUIRED)
#find_package(QtMobility)
#INCLUDE(${QT_USE_FILE})

QT4_WRAP_UI(UISrcs main.ui)
QT4_WRAP_CPP(MOCSrcs form.h)

include_directories(${include_directories} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

ADD_EXECUTABLE(Message Message.cpp form.h form.cpp ${UISrcs} ${MOCSrcs})
TARGET_LINK_LIBRARIES(Message ${QT_LIBRARIES} ${QTMOBILITY})

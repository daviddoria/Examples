cmake_minimum_required(VERSION 2.6)

PROJECT(SqlQueryModel)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

include_directories(${include_directories} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
INCLUDE_DIRECTORIES(${QT_QTSQL_INCLUDE_DIR})

QT4_WRAP_UI(UISrcs SqlQueryModel.ui)
QT4_WRAP_CPP(MOCSrcs form.h)

ADD_EXECUTABLE(SqlQueryModel SqlQueryModel.cpp form.cpp ${MOCSrcs} ${UISrcs})
TARGET_LINK_LIBRARIES(SqlQueryModel ${QT_LIBRARIES} ${QT_QTSQL_LIBRARY_RELEASE} )

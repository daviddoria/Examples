cmake_minimum_required(VERSION 2.6)

PROJECT(ListViewSubclass)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

QT4_WRAP_UI(ListViewSubclass_UI form.ui)
QT4_WRAP_CPP(ListViewSubclass_MOC form.h CustomListView.h)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

ADD_EXECUTABLE(ListViewSubclass main.cpp form.h form.cpp CustomListView.cpp ${ListViewSubclass_UI} ${ListViewSubclass_MOC})
TARGET_LINK_LIBRARIES(ListViewSubclass ${QT_LIBRARIES})

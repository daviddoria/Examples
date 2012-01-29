cmake_minimum_required(VERSION 2.6)

PROJECT(ItemSelectionModel)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

QT4_WRAP_UI(ItemSelectionModel_UI form.ui)
QT4_WRAP_CPP(ItemSelectionModel_MOC form.h)

include_directories(${include_directories} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

ADD_EXECUTABLE(ItemSelectionModel main.cpp form.h form.cpp ${ItemSelectionModel_UI} ${ItemSelectionModel_MOC})
TARGET_LINK_LIBRARIES(ItemSelectionModel ${QT_LIBRARIES})

cmake_minimum_required(VERSION 2.6)

PROJECT(String)

find_package(wxWidgets COMPONENTS core base REQUIRED)
include( ${wxWidgets_USE_FILE} )

ADD_EXECUTABLE(Filename Filename.cpp )
TARGET_LINK_LIBRARIES(Filename ${wxWidgets_LIBRARIES})

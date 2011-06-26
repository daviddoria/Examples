cmake_minimum_required(VERSION 2.6)

PROJECT(String)

find_package(wxWidgets COMPONENTS core base REQUIRED)
include( ${wxWidgets_USE_FILE} )

ADD_EXECUTABLE(String String.cpp )
TARGET_LINK_LIBRARIES(String ${wxWidgets_LIBRARIES})

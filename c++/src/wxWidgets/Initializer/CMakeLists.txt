cmake_minimum_required(VERSION 2.6)

PROJECT(Initializer)

find_package(wxWidgets COMPONENTS core base REQUIRED)
include( ${wxWidgets_USE_FILE} )

ADD_EXECUTABLE(Initializer Initializer.cpp )
TARGET_LINK_LIBRARIES(Initializer ${wxWidgets_LIBRARIES})

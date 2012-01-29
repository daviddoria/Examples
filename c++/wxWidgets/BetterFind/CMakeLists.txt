cmake_minimum_required(VERSION 2.6)

PROJECT(FindWithVersion)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/" ${CMAKE_MODULE_PATH})

find_package(wxWidgets 2.9 COMPONENTS core base REQUIRED)
if(wxWidgets_FOUND)
 message("wxWidgets found successfully.")
 include( ${wxWidgets_USE_FILE} )
else()
 message(FATAL_ERROR "wxWidgets was not found!")
endif()

if(wxWidgets_PFVERSION LESS 2.9)
 message(FATAL_ERROR "wxWidgets is not a high enough version!")
else()
 message("wxWidgets_PFVERSION ${wxWidgets_PFVERSION}")
endif()

ADD_EXECUTABLE(FindWithVersion FindWithVersion.cpp )
TARGET_LINK_LIBRARIES(FindWithVersion ${wxWidgets_LIBRARIES})

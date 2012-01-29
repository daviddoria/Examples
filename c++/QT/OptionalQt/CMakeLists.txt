cmake_minimum_required(VERSION 2.6)

PROJECT(OptionalQt)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

QT4_WRAP_UI(UISrcs form.ui)
QT4_WRAP_CPP(MOCSrcs form.h)

include_directories(${include_directories} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

add_executable(interactive interactive.cpp form.h form.cpp ${MOCSrcs} ${UISrcs})
set_target_properties(interactive PROPERTIES COMPILE_FLAGS "-DINTERACTIVE")
TARGET_LINK_LIBRARIES(interactive ${QT_LIBRARIES})

add_executable(noninteractive noninteractive.cpp form.h form.cpp ${MOCSrcs} ${UISrcs})
TARGET_LINK_LIBRARIES(noninteractive ${QT_LIBRARIES})


cmake_minimum_required(VERSION 2.6)

PROJECT(JetColormapFunctor)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(JetColormapFunctor JetColormapFunctor.cxx)
TARGET_LINK_LIBRARIES(JetColormapFunctor ITKIO)

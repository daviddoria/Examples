cmake_minimum_required(VERSION 2.6)

PROJECT(MetaDataDictionary)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(MetaDataDictionary MetaDataDictionary.cxx)
TARGET_LINK_LIBRARIES(MetaDataDictionary ITKCommon ITKIO)


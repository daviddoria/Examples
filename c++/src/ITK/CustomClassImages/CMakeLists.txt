cmake_minimum_required(VERSION 2.6)

PROJECT(CustomClassImages)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(CustomClassImages CustomClassImages.cpp)
TARGET_LINK_LIBRARIES(CustomClassImages
ITKIO
)
#
# ADD_EXECUTABLE(ReadWriteCustom TestClass.cpp ReadWriteCustom.cpp)
# TARGET_LINK_LIBRARIES(ReadWriteCustom
# ITKIO
# )
#
# ADD_EXECUTABLE(ReadWriteStandard ReadWriteStandard.cpp)
# TARGET_LINK_LIBRARIES(ReadWriteStandard
# ITKIO
# )
#
# ADD_EXECUTABLE(Test Test.cpp)
# TARGET_LINK_LIBRARIES(Test
# ITKIO
# )

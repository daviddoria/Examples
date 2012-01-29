cmake_minimum_required(VERSION 2.6)

PROJECT(Concurrent)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

include_directories(${include_directories} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

ADD_EXECUTABLE(BlockingMapMemberFunction BlockingMapMemberFunction.cpp)
TARGET_LINK_LIBRARIES(BlockingMapMemberFunction ${QT_LIBRARIES})

ADD_EXECUTABLE(BlockingMappedMemberFunction BlockingMappedMemberFunction.cpp)
TARGET_LINK_LIBRARIES(BlockingMappedMemberFunction ${QT_LIBRARIES})

ADD_EXECUTABLE(BlockingMappedMemberFunctionOtherObjectBind BlockingMappedMemberFunctionOtherObjectBind.cpp)
TARGET_LINK_LIBRARIES(BlockingMappedMemberFunctionOtherObjectBind ${QT_LIBRARIES})

ADD_EXECUTABLE(BlockingMappedMemberFunctionOtherObjectOperator BlockingMappedMemberFunctionOtherObjectOperator.cpp)
TARGET_LINK_LIBRARIES(BlockingMappedMemberFunctionOtherObjectOperator ${QT_LIBRARIES})

# ADD_EXECUTABLE(Concurrent Concurrent.cpp)# ${MOCSrcs} ${UISrcs})
# TARGET_LINK_LIBRARIES(Concurrent ${QT_LIBRARIES})
# 
# ADD_EXECUTABLE(Inplace Inplace.cpp)# ${MOCSrcs} ${UISrcs})
# TARGET_LINK_LIBRARIES(Inplace ${QT_LIBRARIES})
# 
# ADD_EXECUTABLE(InplaceComplexType InplaceComplexType.cpp)# ${MOCSrcs} ${UISrcs})
# TARGET_LINK_LIBRARIES(InplaceComplexType ${QT_LIBRARIES})
# 
# ADD_EXECUTABLE(MemberFunction MemberFunction.cpp)# ${MOCSrcs} ${UISrcs})
# TARGET_LINK_LIBRARIES(MemberFunction ${QT_LIBRARIES})
# 
# ADD_EXECUTABLE(NonBlockingNonStaticMap NonBlockingNonStaticMap.cpp)
# TARGET_LINK_LIBRARIES(NonBlockingNonStaticMap ${QT_LIBRARIES})
# 
# ADD_EXECUTABLE(NonStaticMapAnyClassMember NonStaticMapAnyClassMember.cpp)
# TARGET_LINK_LIBRARIES(NonStaticMapAnyClassMember ${QT_LIBRARIES})



# ADD_EXECUTABLE(NonBlockingNonStaticMapped NonBlockingNonStaticMapped.cpp)
# TARGET_LINK_LIBRARIES(NonBlockingNonStaticMapped ${QT_LIBRARIES})

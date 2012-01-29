# - Find a wxWidgets (a.k.a., wxWindows) installation.
# This module finds if wxWidgets is installed and selects a default
# configuration to use.
#
# The following variables are searched for and set to defaults in case
# of multiple choices. Change them if the defaults are not desired:
#
#  wxWidgets_ROOT_DIR      - Base wxWidgets directory
#                            (e.g., C:/wxWidgets-2.6.3).
#  wxWidgets_LIB_DIR       - Path to wxWidgets libraries
#                            (e.g., C:/wxWidgets-2.6.3/lib/${s_wxCompiler}_lib).
#  wxWidgets_CONFIGURATION - Configuration to use
#                            (e.g., msw, mswd, mswu, mswunivud, etc.)
#  wxWidgets_FIND_COMPONENTS      - Libraries to use besides the common
#                            required ones; set to base and core by
#                            default. You could also list them in
#                            FIND_PACKAGE(wxWidgets REQUIRED
#                                         <components>)
#
# The following are set after configuration is done:
#
#  wxWidgets_FOUND            - Set to TRUE if wxWidgets was found.
#  wxWidgets_INCLUDE_DIRS     - Include directories for WIN32
#                               i.e., where to find "wx/wx.h" and
#                               "wx/setup.h"; possibly empty for unices.
#  wxWidgets_LIBRARIES        - Path to the wxWidgets libraries.
#  wxWidgets_LIBRARY_DIRS     - compile time link dirs, useful for
#                               rpath on UNIX. Typically an empty string
#                               in WIN32 environment.
#  wxWidgets_DEFINITIONS      - Contains defines required to compile/link
#                               against WX, e.g. -DWXUSINGDLL
#  wxWidgets_CXX_FLAGS        - Include dirs and ompiler flags for
#                               unices, empty on WIN32. Esentially
#                               "`wx-config --cxxflags`".
#  wxWidgets_USE_FILE         - convenience include file
#
#  wxWidgets_PFVERSION        - version of wxWidgets that was found
#  wxWidgets_PORT             - port of wxWidgets ( msw, gtk2, gtk etc. )
#  wxWidgets_UNV              - contains "unv" when universal wxWidgets is required
#  wxWidgets_UCD              - contains "u" when unicode is required
#  wxWidgets_DBG              - contains "d" when debug is required
#
# Sample usage:
#
#   set(wxWidgets_USE_LIBS base core gl net)
#   FIND_PACKAGE(wxWidgets)
#   if(wxWidgets_FOUND)
#     INCLUDE(${wxWidgets_USE_FILE})
#     # and for each of your dependant executable/library targets:
#     TARGET_LINK_LIBRARIES(<YourTarget> ${wxWidgets_LIBRARIES})
#   endif(wxWidgets_FOUND)
#
# Sample usage with monolithic wx build:
#   set(wxWidgets_USE_LIBS msw26 expat jpeg gl png regex tiff zlib)
#   ...

# NOTES
#
# This module has been tested on the WIN32 platform with wxWidgets
# 2.6.2, 2.6.3, and 2.5.3. However, it has been designed to
# easily extend support to all possible builds, e.g., static/shared,
# debug/release, unicode, universal, multilib/monolithic, etc..
#
# If you want to use the module and your build type is not supported
# out-of-the-box, please contact me to exchange information on how
# your system is setup and I'll try to add support for it.
#
# AUTHOR
#
# Miguel A. Figueroa-Villanueva (miguelf at ieee dot org).
# Jan Woetzel (jw at mip.informatik.uni-kiel.de).
#
# Based on previous works of:
# Jan Woetzel (FindwxWindows.cmake),
# Jorgen Bodde and Jerry Fath (FindwxWin.cmake).
#
# modified by Klaas Holwerda
#
# TODO/ideas
#
# (1) Option/Setting to use all available wx libs
# In contrast to expert developer who lists the
# minimal set of required libs in wxWidgets_USE_LIBS
# there is the newbie user:
#   - who just wants to link against WX with more 'magic'
#   - doesn't know the internal structure of WX or how it was built,
#     in particular if it is monolithic or not
#   - want to link against all available WX libs
# Basically, the intent here is to mimic what wx-config would do by
# default (i.e., `wx-config --libs`).
#
# Possible solution:
#   Add a reserved keyword "std" that initializes to what wx-config
# would default to. If the user has not set the wxWidgets_USE_LIBS,
# default to "std" instead of "base core" as it is now. To implement
# "std" will basically boil down to a FOR_EACH lib-FOUND, but maybe
# checking whether a minimal set was found.
#


#
# Helper macro to control the debugging output globally.
# - NOTE: This and all the DBG_MSG calls should be removed after the
#         module stabilizes.
#
macro(DBG_MSG p_msg)
  # MESSAGE(STATUS "${CMAKE_CURRENT_LIST_FILE}(${CMAKE_CURRENT_LIST_LINE}): ${p_msg}")
  # MESSAGE( "${CMAKE_CURRENT_LIST_FILE}(${CMAKE_CURRENT_LIST_LINE}): ${p_msg}")
endmacro(DBG_MSG)

#
# conditional 
# - NOTE: This and all the DBG_MSG calls should be removed after the
#         module stabilizes.
#
macro( ERROR_MSG type p_msg)
    # make FIND_PACKAGE friendly
    if(NOT wxWidgets_FIND_QUIETLY)
        if(wxWidgets_FIND_REQUIRED)
            MESSAGE( ${type} "${CMAKE_CURRENT_LIST_FILE}(${CMAKE_CURRENT_LIST_LINE}): ${p_msg}")
        else(wxWidgets_FIND_REQUIRED)
            MESSAGE( ${type} "ERROR: ${CMAKE_CURRENT_LIST_FILE}(${CMAKE_CURRENT_LIST_LINE}): ${p_msg}")
        endif(wxWidgets_FIND_REQUIRED)
    endif(NOT wxWidgets_FIND_QUIETLY)
endmacro( ERROR_MSG )

# find wxWidgets_ROOT_DIR ( + wxWidgets_CONFIG_EXECUTABLE on Unix )
# - on windows using WXWIN or wxWidgets_ROOT_DIR variable or registry
# - on Unix using wx-config from the path or WXWIN or wxWidgets_ROOT_DIR
macro( wx_find_root )
    set( s_wxWidgets_ROOT_DIR "s_wxWidgets_ROOT_DIR-NOTFOUND" CACHE INTERNAL "Cleared." FORCE )
    mark_as_advanced( FORCE s_wxWidgets_ROOT_DIR )
    if( ${wxWidgets_FIND_STYLE} STREQUAL "WIN32_STYLE_FIND" )
        if( MINGW AND NOT wxWidgets_MINGW_FROM_SOURCE )           
            find_path( s_wxWidgets_ROOT_DIR 
                NAMES include/common/wx/wx.h
                PATHS
                ${wxWidgets_ROOT_DIR}
                $ENV{WXWIN}
                "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\wxWidgets_is1;Inno Setup: App Path]"  # WX 2.6.x
                DOC "wxWidgets base/installation directory"
            )
        else( MINGW AND NOT wxWidgets_MINGW_FROM_SOURCE )   
            find_path( s_wxWidgets_ROOT_DIR 
                NAMES include/wx/wx.h
                PATHS
                ${wxWidgets_ROOT_DIR}
                $ENV{WXWIN}
                "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\wxWidgets_is1;Inno Setup: App Path]"  # WX 2.6.x
                DOC "wxWidgets base/installation directory"
            )
        endif( MINGW AND NOT wxWidgets_MINGW_FROM_SOURCE )   
    endif()    
        
    if( ${wxWidgets_FIND_STYLE} STREQUAL "UNIX_STYLE_FIND" )
        find_path( s_wxWidgets_ROOT_DIR 
            NAMES wx-config
            PATHS
            $ENV{wxWidgets_ROOT_DIR}
            $ENV{WXWIN}
            DOC "wxWidgets base/installation directory?"
        )
                
        #FIND_PROGRAM(wxWidgets_CONFIG_EXECUTABLE wx-config
        #  ONLY_CMAKE_FIND_ROOT_PATH
        #  )
       
        # if wxWidgets_ROOT_DIR or WXWIN is set, use them, else via PATH
        FIND_PROGRAM(
            wxWidgets_CONFIG_EXECUTABLE
            NAMES wx-config
            PATHS
            s_wxWidgets_ROOT_DIR
        )
    endif()       
        
    if ( NOT s_wxWidgets_ROOT_DIR )
        set( wxWidgets_ROOT_DIR ${s_wxWidgets_ROOT_DIR} CACHE PATH
                            "wxWidgets base/installation directory"  FORCE )                      
        ERROR_MSG( FATAL_ERROR "wxWidgets_ROOT_DIR was not found" )
    else()
        set( wxWidgets_ROOT_DIR ${s_wxWidgets_ROOT_DIR} CACHE PATH
                            "wxWidgets base/installation directory"  FORCE )                      
    endif()    
endmacro( wx_find_root)              

# 
# Set wxWidgets_USE_XX option based on input options or if not defined use the default configuration found
#
macro(wx_config_set_in_options)
    message( STATUS "setting options based on IN else default" )
    if( DEFINED wxWidgets_IN_USE_DEBUG )
        option( wxWidgets_USE_DEBUG  "Use Debug versions of wxWidgets libraries" ${wxWidgets_IN_USE_DEBUG} )
    else( DEFINED wxWidgets_IN_USE_DEBUG )
        option( wxWidgets_USE_DEBUG  "Use Debug versions of wxWidgets libraries" ${wxWidgets_DEFAULT_USE_DEBUG} )
    endif( DEFINED wxWidgets_IN_USE_DEBUG )

    if ( DEFINED wxWidgets_IN_USE_UNICODE )
        option( wxWidgets_USE_UNICODE "Use Unicode versions of wxWidgets libraries" ${wxWidgets_IN_USE_UNICODE} )
    else( DEFINED wxWidgets_IN_USE_UNICODE )        
        option( wxWidgets_USE_UNICODE "Use Unicode versions of wxWidgets libraries" ${wxWidgets_DEFAULT_USE_UNICODE} )
    endif ( DEFINED wxWidgets_IN_USE_UNICODE )        

    if( DEFINED wxWidgets_IN_USE_STATIC )
        option( wxWidgets_USE_STATIC "Use static versions (.a) of wxWidgets libraries" ${wxWidgets_IN_USE_STATIC} )
    else( DEFINED wxWidgets_IN_USE_STATIC )
        option( wxWidgets_USE_STATIC "Use static versions (.a) of wxWidgets libraries" ${wxWidgets_DEFAULT_USE_STATIC} )
    endif( DEFINED wxWidgets_IN_USE_STATIC )

    if( DEFINED wxWidgets_IN_USE_UNIVERSAL )
        option( wxWidgets_USE_UNIVERSAL "Use Universal versions (.so) of wxWidgets libraries" ${wxWidgets_IN_USE_UNIVERSAL} )
    else( DEFINED wxWidgets_IN_USE_UNIVERSAL )
        option( wxWidgets_USE_UNIVERSAL "Use Universal versions (.so) of wxWidgets libraries" ${wxWidgets_DEFAULT_USE_UNIVERSAL} )
    endif( DEFINED wxWidgets_IN_USE_UNIVERSAL )

    if( DEFINED wxWidgets_IN_USE_MONOLITHIC )
        option( wxWidgets_USE_MONOLITHIC "Use wxWidgets build as monolithic library" ${wxWidgets_IN_USE_MONOLITHIC} )
    else( DEFINED wxWidgets_IN_USE_MONOLITHIC )
        option( wxWidgets_USE_MONOLITHIC "Use wxWidgets build as monolithic library" ${wxWidgets_DEFAULT_USE_MONOLITHIC} )
    endif( DEFINED wxWidgets_IN_USE_MONOLITHIC )

    if( DEFINED wxWidgets_IN_PFVERSION )
        set( wxWidgets_PFVERSION ${wxWidgets_IN_PFVERSION} CACHE STRING "Set wxWidgets version"  FORCE )
    else( DEFINED wxWidgets_IN_PFVERSION )
        set( wxWidgets_PFVERSION ${wxWidgets_DEFAULT_PFVERSION} CACHE STRING "Set wxWidgets version"  FORCE )
    endif( DEFINED wxWidgets_IN_PFVERSION )

    if( DEFINED wxWidgets_IN_PORT )
        set( wxWidgets_PORT ${wxWidgets_IN_PORT} CACHE STRING "Set wxWidgets configuration port"  FORCE )    
    else( DEFINED wxWidgets_IN_PORT )
        set( wxWidgets_PORT ${wxWidgets_DEFAULT_PORT} CACHE STRING "Set wxWidgets configuration port"  FORCE )    
    endif( DEFINED wxWidgets_IN_PORT )
endmacro(wx_config_set_in_options)

# find a valid compiled configuration for the Cmake Generator in use.
# - on windows this is checking directories on the root of wxWidgets in its lib directory
# - on Unix "wx-config --selected-config" is used
macro( wx_find_defaultconfig p_wxRootDir )
    if( NOT ${p_wxRootDir} STREQUAL "" )
        if( ${wxWidgets_FIND_STYLE} STREQUAL "WIN32_STYLE_FIND" )
            # select one default tree inside the already determined wx tree
            # prefer static/shared order usually consistent with build settings
            if( MINGW AND NOT wxWidgets_MINGW_FROM_SOURCE )
                find_path( l_wxLibDir
                    NAMES ${s_libPre}wxpng.${s_libExt}
                    PATHS
                    ${p_wxRootDir}/lib
                    DOC "Path to wxWidgets libraries?"
                    NO_DEFAULT_PATH
                )
            else( MINGW AND NOT wxWidgets_MINGW_FROM_SOURCE )
                set( wxWidgets_DEFAULT_USE_STATIC OFF )
                find_path( l_wxLibDir
                    NAMES ${s_libPre}wxpng.${s_libExt} ${s_libPre}wxpngd.${s_libExt}
                    PATHS
                    ${p_wxRootDir}/lib/${s_wxCompiler}_lib  
                    DOC "Path to wxWidgets libraries?"
                    NO_DEFAULT_PATH
                )
                if( l_wxLibDir )
                    set( wxWidgets_DEFAULT_USE_STATIC ON )
                endif()
                find_path( l_wxLibDir
                    NAMES ${s_libPre}wxpng.${s_libExt} ${s_libPre}wxpngd.${s_libExt}
                    PATHS
                    ${p_wxRootDir}/lib/${s_wxCompiler}_dll   
                    DOC "Path to wxWidgets libraries?"
                    NO_DEFAULT_PATH
                )
            endif( MINGW AND NOT wxWidgets_MINGW_FROM_SOURCE )
            mark_as_advanced( FORCE l_wxLibDir )
            if ( NOT l_wxLibDir )
                    ERROR_MSG( FATAL_ERROR "l_wxLibDir was not found")
            endif ()

            set( wxWidgets_DEFAULT_PORT "msw" )
            if( EXISTS ${l_wxLibDir}/mswunivud )
                set( wxWidgets_DEFAULT_SELECTED_CONFIG mswunivud )
                set( wxWidgets_DEFAULT_USE_UNIVERSAL ON )
                set( wxWidgets_DEFAULT_USE_UNICODE ON )
                set( wxWidgets_DEFAULT_USE_DEBUG ON )
            elseif( EXISTS ${l_wxLibDir}/mswunivd )
                set( wxWidgets_DEFAULT_SELECTED_CONFIG mswunivd )
                set( wxWidgets_DEFAULT_USE_UNIVERSAL ON )
                set( wxWidgets_DEFAULT_USE_UNICODE OFF )
                set( wxWidgets_DEFAULT_USE_DEBUG ON )
            elseif( EXISTS ${l_wxLibDir}/mswud )
                set( wxWidgets_DEFAULT_SELECTED_CONFIG mswud )
                set( wxWidgets_DEFAULT_USE_UNIVERSAL OFF )
                set( wxWidgets_DEFAULT_USE_UNICODE ON )
                set( wxWidgets_DEFAULT_USE_DEBUG ON )
            elseif( EXISTS ${l_wxLibDir}/mswd )
                set( wxWidgets_DEFAULT_SELECTED_CONFIG mswd )
                set( wxWidgets_DEFAULT_USE_UNIVERSAL OFF )
                set( wxWidgets_DEFAULT_USE_UNICODE OFF )
                set( wxWidgets_DEFAULT_USE_DEBUG ON )
            elseif( EXISTS ${l_wxLibDir}/mswunivu )
                set( wxWidgets_DEFAULT_SELECTED_CONFIG mswunivu )
                set( wxWidgets_DEFAULT_USE_UNIVERSAL ON )
                set( wxWidgets_DEFAULT_USE_UNICODE ON )
                set( wxWidgets_DEFAULT_USE_DEBUG OFF )
            elseif( EXISTS ${l_wxLibDir}/mswuniv )
                set( wxWidgets_DEFAULT_SELECTED_CONFIG mswuniv )
                set( wxWidgets_DEFAULT_USE_UNIVERSAL ON )
                set( wxWidgets_DEFAULT_USE_UNICODE OFF )
                set( wxWidgets_DEFAULT_USE_DEBUG OFF )
            elseif( EXISTS ${l_wxLibDir}/mswu )
                set( wxWidgets_DEFAULT_SELECTED_CONFIG mswu )
                set( wxWidgets_DEFAULT_USE_UNIVERSAL OFF )
                set( wxWidgets_DEFAULT_USE_UNICODE ON )
                set( wxWidgets_DEFAULT_USE_DEBUG OFF )
            elseif( EXISTS ${l_wxLibDir}/msw )
                set( wxWidgets_DEFAULT_SELECTED_CONFIG msw )
                set( wxWidgets_DEFAULT_USE_UNIVERSAL OFF )
                set( wxWidgets_DEFAULT_USE_UNICODE OFF )
                set( wxWidgets_DEFAULT_USE_DEBUG OFF )
            else()
                set( wxWidgets_DEFAULT_SELECTED_CONFIG )
                if ( wxWidgets_FIND_REQUIRED )
                    ERROR_MSG( FATAL_ERROR "no default configuration found in: ${p_wxRootDir}")                                
                endif ()    
            endif()

            if( NOT ${wxWidgets_DEFAULT_SELECTED_CONFIG} STREQUAL "" )
                wx_config_set_in_options()
            else()    
                set( wxWidgets_FOUND FALSE)
                foreach(_upper_opt_name DEBUG STATIC UNICODE UNIVERSAL)
                    set(wxWidgets_DEFAULT_USE_${_upper_opt_name} OFF)
                endforeach(_upper_opt_name)
                set( wxWidgets_DEFAULT_PORT ""  )    
                set( wxWidgets_DEFAULT_PFVERSION ""  )                   
            endif()
            
            set( wxWidgets_DEFAULT_SELECTED_CONFIG ${wxWidgets_DEFAULT_SELECTED_CONFIG} CACHE STRING
                                "Default Selected wxWidgets configuration"  FORCE )                      
            mark_as_advanced( FORCE wxWidgets_DEFAULT_SELECTED_CONFIG )
        endif()

        if( ${wxWidgets_FIND_STYLE} STREQUAL "UNIX_STYLE_FIND" )
            #
            # Set the default values based on "wx-config --selected-config".
            #
            execute_process(
                COMMAND sh "${wxWidgets_CONFIG_EXECUTABLE}" --selected-config
                OUTPUT_VARIABLE wxWidgets_DEFAULT_SELECTED_CONFIG
                RESULT_VARIABLE _wx_result
                ERROR_QUIET
                OUTPUT_STRIP_TRAILING_WHITESPACE
            )
            if( _wx_result EQUAL 0)
                set( wxWidgets_DEFAULT_SELECTED_CONFIG ${wxWidgets_DEFAULT_SELECTED_CONFIG} CACHE STRING
                                    "Default Selected wxWidgets configuration"  FORCE )
                mark_as_advanced( FORCE wxWidgets_DEFAULT_SELECTED_CONFIG )
                foreach(_opt_name debug static unicode universal)
                    string(TOUPPER ${_opt_name} _upper_opt_name)
                    if( wxWidgets_DEFAULT_SELECTED_CONFIG MATCHES ".*${_opt_name}.*")
                        set(wxWidgets_DEFAULT_USE_${_upper_opt_name} ON )
                    else( wxWidgets_DEFAULT_SELECTED_CONFIG MATCHES ".*${_opt_name}.*")
                        set(wxWidgets_DEFAULT_USE_${_upper_opt_name} OFF )
                    endif( wxWidgets_DEFAULT_SELECTED_CONFIG MATCHES ".*${_opt_name}.*")
                endforeach(_opt_name)
                
                # based on this output figure out the default wxWidgets_DEFAULT_PORT, which can be set different by user. 
                string( REGEX REPLACE "^([^\\-]+).*" "\\1" wxWidgets_DEFAULT_PORT "${wxWidgets_DEFAULT_SELECTED_CONFIG}" )
                # gtk|gtk2|x11|motif|dfb|mac|mgl        
                # string( REGEX MATCH "([0-9].[0-9]).[0-9]*" wxWidgets_DEFAULT_PFVERSION "${wxWidgets_DEFAULT_SELECTED_CONFIG}" )
                string( REGEX MATCH "([0-9].[0-9]).*" wxWidgets_DEFAULT_PFVERSION "${wxWidgets_DEFAULT_SELECTED_CONFIG}" )

                wx_config_set_in_options()

            else( _wx_result EQUAL 0)
                set( wxWidgets_FOUND FALSE)
                foreach(_upper_opt_name DEBUG STATIC UNICODE UNIVERSAL)
                    set(wxWidgets_DEFAULT_USE_${_upper_opt_name} OFF)
                endforeach(_upper_opt_name)
                set( wxWidgets_DEFAULT_PORT ""  )    
                set( wxWidgets_DEFAULT_PFVERSION ""  )   
                if ( wxWidgets_FIND_REQUIRED )
                    ERROR_MSG( FATAL_ERROR "no default configuration found: ${wxWidgets_CONFIG_EXECUTABLE} --selected_config failed")                    
                endif ()
            endif( _wx_result EQUAL 0)
        endif()
    else()
          ERROR_MSG( FATAL_ERROR "No input for wxWidgets root directory")                    
    endif()
endmacro( wx_find_defaultconfig )

#
set( wxWidgets_FOUND FALSE)
set( wxWidgets_INCLUDE_DIRS      "" )
set( wxWidgets_LIBRARIES        "" )
set( wxWidgets_LIBRARY_DIRS "" )
#NOT DO set( wxWidgets_LIB_DIR "" )
set( wxWidgets_CXX_FLAGS        "" )
set( wxWidgets_DEFINITIONS "" )
set( wxWidgets_UNV "" )
set( wxWidgets_UCD "" )
set( wxWidgets_DBG "" )

set( wxWidgets_DEFAULT_PORT "" )
set( wxWidgets_DEFAULT_PFVERSION "" )
set( wxWidgets_DEFAULT_USE_UNIVERSAL OFF )
set( wxWidgets_DEFAULT_USE_UNICODE ON )
set( wxWidgets_DEFAULT_USE_DEBUG OFF )


# Using SYSTEM with INCLUDE_DIRECTORIES in conjunction with wxWidgets on
# the Mac produces compiler errors. Set wxWidgets_INCLUDE_DIRS_NO_SYSTEM
# to prevent UsewxWidgets.cmake from using SYSTEM.
#
# See cmake mailing list discussions for more info:
#   http://www.cmake.org/pipermail/cmake/2008-April/021115.html
#   http://www.cmake.org/pipermail/cmake/2008-April/021146.html
#
if(APPLE)
  set(wxWidgets_INCLUDE_DIRS_NO_SYSTEM 1)
endif(APPLE)

# DEPRECATED: This is a patch to support the DEPRECATED use of
# wxWidgets_USE_LIBS.
#
# If wxWidgets_USE_LIBS is set:
# - if using <components>, then override wxWidgets_USE_LIBS
# - else set wxWidgets_FIND_COMPONENTS to wxWidgets_USE_LIBS
if(wxWidgets_USE_LIBS AND NOT wxWidgets_FIND_COMPONENTS)
  set(wxWidgets_FIND_COMPONENTS ${wxWidgets_USE_LIBS})
endif(wxWidgets_USE_LIBS AND NOT wxWidgets_FIND_COMPONENTS)
dbg_msg("wxWidgets_FIND_COMPONENTS : ${wxWidgets_FIND_COMPONENTS}")
   
#=====================================================================
#=====================================================================
set( wxWidgets_FIND_STYLE "NOT_DEFINED_STYLE_FIND" CACHE STRING "wxWidgets find style" FORCE )
mark_as_advanced( FORCE wxWidgets_FIND_STYLE )

if ( ${CMAKE_GENERATOR} STREQUAL "MSYS Makefiles" )
    # remove the standard -O3 option, it does not work
    set( wxWidgets_FIND_STYLE "UNIX_STYLE_FIND" CACHE STRING "wxWidgets find style" FORCE )
        
    SET (CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG" CACHE STRING
        "Flags used by the compiler during release builds" FORCE)
    SET (CMAKE_CX_FLAGS_RELEASE "-DNDEBUG" CACHE STRING
        "Flags used by the compiler during release builds" FORCE)    
        
    set(wxWidgets_FIND_COMPONENTS ${wxWidgets_FIND_COMPONENTS} aui qa xrc html adv xml net core base )

endif ( ${CMAKE_GENERATOR} STREQUAL "MSYS Makefiles" )

if ( ${CMAKE_GENERATOR} STREQUAL "MinGW Makefiles" )
    option( wxWidgets_MINGW_FROM_SOURCE  "Use wxWidgets source distribution instead of wxDevCpp devpack" ON)
    # remove the standard -O3 option, it does not work
    set( wxWidgets_FIND_STYLE "WIN32_STYLE_FIND" CACHE STRING "wxWidgets find style" FORCE )   
         
    SET (CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG" CACHE STRING
        "Flags used by the compiler during release builds" FORCE)
    SET (CMAKE_C_FLAGS_RELEASE "-DNDEBUG" CACHE STRING
        "Flags used by the compiler during release builds" FORCE)
    set( wxWidgets_DEFINITIONS ${wxWidgets_DEFINITIONS} -D__WXMSW__ -D__GNUWIN32__ -D__WIN95__ )
    set( wxWidgets_CXX_FLAGS ${wxWidgets_CXX_FLAGS}
        -fexceptions -fno-pcc-struct-return 
        -fstrict-aliasing 
        -Wall    #all warnings
        -Wno-unused-variable
        # -m32  # 32 bits
        #-fexpensive-optimizations 
        #-mwindows  # windows app
        #-mconsole    #console app
    )
    #add some libs which are not already in CMAKE_CXX_STANDARD_LIBRARIES
    set( wxWidgets_complibs
        winmm
        comctl32
        rpcrt4
        wsock32
        kernel32
        user32
        gdi32
        comdlg32
        winspool
        winmm
        shell32
        comctl32
        ole32
        oleaut32
        uuid
        rpcrt4
        advapi32
        wsock32
        odbc32
        opengl32
    )
    
    set(wxWidgets_FIND_COMPONENTS ${wxWidgets_FIND_COMPONENTS} aui xrc html adv xml net core base )
    set( s_wxCompiler "gcc" )
    set( s_libPre "lib" )
    set( s_libExt "a" )           
endif ( ${CMAKE_GENERATOR} STREQUAL "MinGW Makefiles" )

if ( ${CMAKE_GENERATOR} STREQUAL "Unix Makefiles" )
    set( wxWidgets_FIND_STYLE "UNIX_STYLE_FIND" CACHE STRING "wxWidgets find style" FORCE )

    if( CYGWIN OR MINGW )
        SET (CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG" CACHE STRING
            "Flags used by the compiler during release builds" FORCE)
        SET (CMAKE_C_FLAGS_RELEASE "-DNDEBUG" CACHE STRING
            "Flags used by the compiler during release builds" FORCE)
    endif( CYGWIN OR MINGW )
    
    set(wxWidgets_FIND_COMPONENTS ${wxWidgets_FIND_COMPONENTS} aui qa xrc html adv xml net core base )
    set( s_wxCompiler "gcc" )
    set( s_libPre "lib" )
    set( s_libExt "a" )              
endif ( ${CMAKE_GENERATOR} STREQUAL "Unix Makefiles" )

if ( ${CMAKE_GENERATOR} MATCHES "Visual Studio.*" )
    set( wxWidgets_FIND_STYLE "WIN32_STYLE_FIND" CACHE STRING "wxWidgets find style" FORCE )    
    #add some libs which are not already in CMAKE_CXX_STANDARD_LIBRARIES
    set( wxWidgets_complibs 
        winmm
        comctl32
        rpcrt4
        wsock32
    )

    set(wxWidgets_FIND_COMPONENTS ${wxWidgets_FIND_COMPONENTS} aui qa xrc html adv xml net core base )
    set( s_wxCompiler "vc" )
    set( s_libPre "" )
    set( s_libExt "lib" )           
endif ( ${CMAKE_GENERATOR} MATCHES "Visual Studio.*" )

if ( ${CMAKE_GENERATOR} MATCHES "NMake Makefiles" )
    set( wxWidgets_FIND_STYLE "WIN32_STYLE_FIND" CACHE STRING "wxWidgets find style" FORCE )    
    #add some libs which are not already in CMAKE_CXX_STANDARD_LIBRARIES
    set( wxWidgets_complibs 
        winmm
        comctl32
        rpcrt4
        wsock32
    )
    set(wxWidgets_FIND_COMPONENTS ${wxWidgets_FIND_COMPONENTS} aui qa xrc html adv xml net core base )
    set( s_wxCompiler "vc" )
    set( s_libPre "" )
    set( s_libExt "lib" )        
endif ()

if ( ${CMAKE_GENERATOR} MATCHES "Borland Makefiles" )
    set( wxWidgets_FIND_STYLE "WIN32_STYLE_FIND" CACHE STRING "wxWidgets find style" FORCE )    
    #add some libs which are not already in CMAKE_CXX_STANDARD_LIBRARIES
    set( wxWidgets_complibs 
        winmm
        comctl32
        rpcrt4
        wsock32
    )
    
    set( wxWidgets_CXX_FLAGS ${wxWidgets_CXX_FLAGS}
    -tWR -a8 -g0
    )   

    set(wxWidgets_FIND_COMPONENTS ${wxWidgets_FIND_COMPONENTS} aui xrc html adv xml net core )
    set( s_wxCompiler "bcc" )
    set( s_libPre "" )
    set( s_libExt "lib" )            
endif ( ${CMAKE_GENERATOR} MATCHES "Borland Makefiles" )

if( "${CMAKE_CXX_COMPILER}" MATCHES ".*wcl.*" )
    set( s_wxCompiler "wat" )
    set( s_libPre "" )
    set( s_libExt "lib" )        
endif( "${CMAKE_CXX_COMPILER}" MATCHES ".*wcl.*" )

#MESSAGE( "generator: ${CMAKE_GENERATOR}")
mark_as_advanced( FORCE wxWidgets_FIND_STYLE )

#=====================================================================
# windows style find
#=====================================================================
if( ${wxWidgets_FIND_STYLE} STREQUAL "WIN32_STYLE_FIND" )
    set(wxWidgets_FOUND FALSE)

    set( wxWidgets_PORT "msw" CACHE STRING
                "Set wxWidgets configuration (${wxWidgets_PORT})"  FORCE )            
    mark_as_advanced( FORCE wxWidgets_PORT )
   
    wx_find_root() 
    
    # If wxWidgets_ROOT_DIR changed, clear all libraries and lib dir.
    if(NOT s_wxRootDir STREQUAL wxWidgets_ROOT_DIR)
        set(s_wxRootDir ${wxWidgets_ROOT_DIR} CACHE INTERNAL "wxWidgets_ROOT_DIR")
        #  WX_CLEAR_ALL_DBG_LIBS()
        #  WX_CLEAR_ALL_REL_LIBS()
        set(wxWidgets_LIB_DIR "wxWidgets_LIB_DIR-NOTFOUND" CACHE PATH "Cleared." FORCE)
    endif(NOT s_wxRootDir STREQUAL wxWidgets_ROOT_DIR)
    
    if ( NOT DEFINED wxWidgets_DEFAULT_SELECTED_CONFIG )
        wx_find_defaultconfig( ${wxWidgets_ROOT_DIR} )
        set( wxWidgets_SELECTED_CONFIG ${wxWidgets_DEFAULT_SELECTED_CONFIG} )
    endif()

    # depending on the options set by the user, a configuration directory in "wxWidgets its root/lib/configuration_directory"
    # can be set. Later on it is checked if that configuration indeed exists. This configuration_directory is unique and holds
    # build.cfg file.
    set( wxWidgets_CONFIGURATION "msw"  )
    
    # we do set these down here according to option above 
    if( wxWidgets_USE_UNIVERSAL )
        set( wxWidgets_UNV "univ" )
        set( wxWidgets_CONFIGURATION "${wxWidgets_CONFIGURATION}univ"  )
    endif( wxWidgets_USE_UNIVERSAL )

    if ( wxWidgets_USE_UNICODE )
        set( wxWidgets_UCD "u" )
        set( wxWidgets_CONFIGURATION "${wxWidgets_CONFIGURATION}u"  )
        set( wxWidgets_DEFINITIONS ${wxWidgets_DEFINITIONS} -DwxUSE_UNICODE  )
    endif ( wxWidgets_USE_UNICODE )        

    if( wxWidgets_USE_DEBUG )
        set( wxWidgets_DBG "d" )
        set( wxWidgets_CONFIGURATION "${wxWidgets_CONFIGURATION}d"  )
        set( wxWidgets_DEFINITIONS ${wxWidgets_DEFINITIONS} -D_DEBUG_  -D__WXDEBUG__  )
    endif( wxWidgets_USE_DEBUG )

    # global settings for std and common wx libs
    # logic could determine _USE_MONOLITHIC automatically
    # but let the user decide for now.
    if( wxWidgets_USE_MONOLITHIC )
        set(wxWidgets_FIND_COMPONENTS ${wxWidgets_FIND_COMPONENTS} mono )
        set(wxWidgets_STD_LIBRARIES mono )
    else (wxWidgets_USE_MONOLITHIC)
        set(wxWidgets_STD_LIBRARIES base core ) # this is default
    endif( wxWidgets_USE_MONOLITHIC )
   
    set( wxWidgets_CONFIGURATION ${wxWidgets_CONFIGURATION} CACHE STRING "wxWidgets configuration" FORCE)
    mark_as_advanced( FORCE wxWidgets_CONFIGURATION )

    set(WX_USE_REL_AND_DBG FALSE)

    #useful common wx libs needed by almost all components
    set(wxWidgets_COMMON_LIBRARIES png tiff jpeg zlib regex expat)

    #---------------------------------------------------------------------
    # WIN32: Helper MACROS
    #---------------------------------------------------------------------

    #
    # Find libraries associated to a configuration.
    # Will set variables in WX_libname(p_dbg) where (p_dbg stand for debug or not)
    # If library is not available, becomes WX_libname-NOTFOUND
    # param: p_unv inuversal wxWidgets prefix
    # param: p_ucd unicode postfix
    # param: p_dbg debug postfix
    macro(WX_FIND_LIBS p_unv p_ucd p_dbg)
        dbg_msg("p_unv = ${p_unv}")
        dbg_msg("p_ucd = ${p_ucd}")
        dbg_msg("p_dbg = ${p_dbg}")

        # Find wxWidgets common libraries
        foreach(LIB png tiff jpeg zlib regex expat)
            set( WX_${LIB}${p_dbg} "WX_${LIB}${p_dbg}-NOTFOUND" CACHE FILEPATH "Cleared." FORCE)
            find_library(WX_${LIB}${p_dbg}
                NAMES
                ${s_libPre}wx${LIB}${p_ucd}${p_dbg} # for regex
                ${s_libPre}wx${LIB}${p_dbg}
                PATHS ${s_wxLibDir}
                NO_DEFAULT_PATH
            )
            get_filename_component( WX_${LIB}${p_dbg} ${WX_${LIB}${p_dbg}} NAME_WE )
            string(REGEX REPLACE "^lib" ""  WX_${LIB}${p_dbg} ${WX_${LIB}${p_dbg}} )         
            mark_as_advanced( WX_${LIB}${p_dbg} )
            dbg_msg("detected ${s_libPre}wx${LIB}${p_ucd}${p_dbg} = ${WX_${LIB}${p_dbg}}")
        endforeach(LIB)

        # Find wxWidgets multilib base libraries
        set( WX_base${p_dbg} "WX_base${p_dbg}-NOTFOUND" CACHE FILEPATH "Cleared." FORCE)
        find_library(WX_base${p_dbg}
            NAMES
            ${s_libPre}wxbase${wxWidgets_PFVERSION}${p_ucd}${p_dbg}
            PATHS ${s_wxLibDir}
            NO_DEFAULT_PATH
        )
        get_filename_component( WX_base${p_dbg} ${WX_base${p_dbg}} NAME_WE )
        string(REGEX REPLACE "^lib" ""  WX_base${p_dbg} ${WX_base${p_dbg}} )         
        mark_as_advanced(WX_base${p_dbg})
        
        foreach(LIB net odbc xml )
            set( WX_${LIB}${p_dbg} "WX_${LIB}${p_dbg}-NOTFOUND" CACHE FILEPATH "Cleared." FORCE)
            find_library(WX_${LIB}${p_dbg}
                NAMES
                ${s_libPre}wxbase${wxWidgets_PFVERSION}${p_ucd}${p_dbg}_${LIB}
                PATHS ${s_wxLibDir}
                NO_DEFAULT_PATH
            )
            get_filename_component( WX_${LIB}${p_dbg} ${WX_${LIB}${p_dbg}} NAME_WE )
            string(REGEX REPLACE "^lib" ""  WX_${LIB}${p_dbg} ${WX_${LIB}${p_dbg}} )         
            mark_as_advanced(WX_${LIB}${p_dbg})
            dbg_msg("detected wxbase${wxWidgets_PFVERSION}${p_ucd}${p_dbg}_${LIB} = ${WX_${LIB}${p_dbg}}")
        endforeach(LIB)
        
        # Find wxWidgets monolithic library
        set( WX_mono${p_dbg} "WX_mono${p_dbg}-NOTFOUND" CACHE FILEPATH "Cleared." FORCE)
        find_library( WX_mono${p_dbg}
            NAMES
            ${s_libPre}wxmsw${p_unv}${wxWidgets_PFVERSION}${p_ucd}${p_dbg}
            PATHS ${s_wxLibDir}
            NO_DEFAULT_PATH
        )
        get_filename_component( WX_mono${p_dbg} ${WX_mono${p_dbg}} NAME_WE )
        string(REGEX REPLACE "^lib" ""  WX_mono${p_dbg} ${WX_mono${p_dbg}} )         
        mark_as_advanced( WX_mono${p_dbg} )
        
        # Find wxWidgets multilib libraries
        foreach( LIB core adv aui html media xrc dbgrid gl qa stc richtext gizmos things netutils gizmos_xrc )
            set( WX_${LIB}${p_dbg} "WX_${LIB}${p_dbg}-NOTFOUND" CACHE FILEPATH "Cleared." FORCE)            
            find_library(WX_${LIB}${p_dbg}
                NAMES
                ${s_libPre}wxmsw${p_unv}${wxWidgets_PFVERSION}${p_ucd}${p_dbg}_${LIB}
                PATHS ${s_wxLibDir}
                NO_DEFAULT_PATH
            )
            get_filename_component( WX_${LIB}${p_dbg} ${WX_${LIB}${p_dbg}} NAME_WE )
            string(REGEX REPLACE "^lib" ""  WX_${LIB}${p_dbg} ${WX_${LIB}${p_dbg}} )         
            mark_as_advanced(WX_${LIB}${p_dbg})
            dbg_msg("detected wxmsw${p_unv}${wxWidgets_PFVERSION}${p_ucd}${p_dbg}_${LIB} = ${WX_${LIB}${p_dbg}}")
        endforeach(LIB)

        #foreach( LIB gl ) the gl lib is now wxmsw28ud_gl.lib, so added above here, and removed here
        foreach( LIB )
            set( WX_${LIB}${p_dbg} "WX_${LIB}${p_dbg}-NOTFOUND" CACHE FILEPATH "Cleared." FORCE)
            find_library(WX_${LIB}${p_dbg}
                NAMES
                ${s_libPre}wxmsw${p_unv}${wxWidgets_PFVERSION}${p_dbg}_${LIB}
                PATHS ${s_wxLibDir}
                NO_DEFAULT_PATH
            )
            get_filename_component( WX_${LIB}${p_dbg} ${WX_${LIB}${p_dbg}} NAME_WE )
            string(REGEX REPLACE "^lib" ""  WX_${LIB}${p_dbg} ${WX_${LIB}${p_dbg}} )         
            mark_as_advanced(WX_${LIB}${p_dbg})
            dbg_msg("detected wxmsw${p_unv}${wxWidgets_PFVERSION}${p_dbg}_${LIB} = ${WX_${LIB}${p_dbg}}")
        endforeach(LIB)
        
    endmacro(WX_FIND_LIBS)

    #
    # Clear all library paths, so that find_library refinds them.
    #
    # Clear a lib, reset its found flag, and mark as advanced.
    macro(wx_clear_lib p_lib)
        set(${p_lib} "${p_lib}-NOTFOUND" CACHE FILEPATH "Cleared." FORCE)
        set(${p_lib}_FOUND FALSE)
        mark_as_advanced(${p_lib})
    endmacro(wx_clear_lib)
  
    # Clear all debug or release library paths (arguments are "d" or "").
    macro(WX_CLEAR_ALL_LIBS p_dbg)
        # Clear wxWidgets common libraries
        foreach(LIB png tiff jpeg zlib regex expat)
            wx_clear_lib(WX_${LIB}${p_dbg})
        endforeach(LIB)
    
        # Clear wxWidgets multilib base libraries
        wx_clear_lib(WX_base${p_dbg})
        foreach(LIB net odbc xml)
            wx_clear_lib(WX_${LIB}${p_dbg})
        endforeach(LIB)
    
        # Clear wxWidgets monolithic library
        wx_clear_lib(WX_mono${p_dbg})
    
        # Clear wxWidgets multilib libraries
        foreach(LIB core adv html media xrc dbgrid gl qa)
            wx_clear_lib(WX_${LIB}${p_dbg})
        endforeach(LIB)
    endmacro(WX_CLEAR_ALL_LIBS)
    
    # Clear all wxWidgets debug libraries.
    macro(WX_CLEAR_ALL_DBG_LIBS)
        WX_CLEAR_ALL_LIBS("d")
    endmacro(WX_CLEAR_ALL_DBG_LIBS)
    # Clear all wxWidgets release libraries.
    macro(WX_CLEAR_ALL_REL_LIBS)
        WX_CLEAR_ALL_LIBS("")
    endmacro(WX_CLEAR_ALL_REL_LIBS)
    
    # check for build.cfg and extract given variable X=Y its value, and set this as a new variable wxWidgets_X with value Y
    macro( find_build_variable p_buildvar )    
        if (EXISTS ${s_wxLibDir}/${wxWidgets_CONFIGURATION}/build.cfg)
            set( wxWidgets_BUILDFILE ${s_wxLibDir}/${wxWidgets_CONFIGURATION}/build.cfg )
            file( STRINGS ${wxWidgets_BUILDFILE} wxWidgets_${p_buildvar} REGEX "^${p_buildvar}=" )
            string(REGEX REPLACE ".*=(.*) " "\\1"  wxWidgets_${p_buildvar} "${wxWidgets_${p_buildvar}}" )
            set( wxWidgets_${p_buildvar} "${wxWidgets_${p_buildvar}}" CACHE STRING "gdiplus used" FORCE )                
            mark_as_advanced( FORCE wxWidgets_${p_buildvar} )
        else (EXISTS ${s_wxLibDir}/${wxWidgets_CONFIGURATION}/build.cfg)
            ERROR_MSG( FATAL_ERROR "WXWIDGET_FOUND FALSE because ${s_wxLibDir}/${wxWidgets_CONFIGURATION}/build.cfg does not exists.")
            set(wxWidgets_FOUND FALSE)
        endif (EXISTS ${s_wxLibDir}/${wxWidgets_CONFIGURATION}/build.cfg)
    endmacro( find_build_variable )

    #
    # Set the wxWidgets_LIBRARIES variable.
    # Also, Sets output variable wxWidgets_FOUND to FALSE if it fails.
    # The variables set in WX_FIND_LIBS are now used to set the wxWidgets_LIBRARIES variable
    # with the right libraries according to build type.
    macro(WX_SET_LIBRARIES p_LIBS p_dbg)
        if(WX_USE_REL_AND_DBG)
            dbg_msg("looking for ${${p_LIBS}}")
            foreach(LIB ${${p_LIBS}})
                dbg_msg("Finding ${LIB} and ${LIB}d")
                dbg_msg("WX_${LIB}  : ${WX_${LIB}}")
                dbg_msg("WX_${LIB}d : ${WX_${LIB}d}")
                if(WX_${LIB} AND WX_${LIB}d)
                    dbg_msg("Found ${LIB} and ${LIB}d")
                    set(wxWidgets_LIBRARIES ${wxWidgets_LIBRARIES}
                        debug     ${WX_${LIB}d}
                        optimized ${WX_${LIB}}
                    )
                else(WX_${LIB} AND WX_${LIB}d)
                    ERROR_MSG( ERROR "- not found due to missing WX_${LIB}=${WX_${LIB}} or WX_${LIB}d=${WX_${LIB}d}")
                    set(wxWidgets_FOUND FALSE)
                endif(WX_${LIB} AND WX_${LIB}d)
            endforeach(LIB)
        else(WX_USE_REL_AND_DBG)
            dbg_msg("looking for ${${p_LIBS}}")
            foreach(LIB ${${p_LIBS}})
                dbg_msg("Finding ${LIB}${p_dbg}")
                dbg_msg("WX_${LIB}${p_dbg} : ${WX_${LIB}${p_dbg}}")
                if(WX_${LIB}${p_dbg})
                    dbg_msg("Found ${LIB}${p_dbg}")
                        set(wxWidgets_LIBRARIES ${wxWidgets_LIBRARIES}
                            ${WX_${LIB}${p_dbg}}
                        )
                else(WX_${LIB}${p_dbg})
                    ERROR_MSG( ERROR "- not found due to missing library  ${LIB}${p_dbg},\nset to WX_${LIB}${p_dbg}=${WX_${LIB}${p_dbg}}")
                    set(wxWidgets_FOUND FALSE)
                endif(WX_${LIB}${p_dbg})
            endforeach(LIB)
        endif(WX_USE_REL_AND_DBG)
        
        foreach(LIB ${${p_LIBS}})
            dbg_msg("required: ${LIB}")
            if(LIB STREQUAL "gl")
                dbg_msg("gl required: ${LIB}")
                set(wxWidgets_LIBRARIES ${wxWidgets_LIBRARIES}
                    opengl32
                    glu32
                )
            endif(LIB STREQUAL "gl")
        endforeach(LIB ${${p_LIBS}})
    
        set(wxWidgets_LIBRARIES ${wxWidgets_LIBRARIES} ${wxWidgets_complibs} )
        
    endmacro(WX_SET_LIBRARIES)

    if(s_wxRootDir)   
        # select one default tree inside the already determined wx tree
        # prefer static/shared order usually consistent with build settings
        if( MINGW AND NOT wxWidgets_MINGW_FROM_SOURCE )
            find_path(wxWidgets_LIB_DIR
                NAMES ${s_libPre}wxpng.${s_libExt}
                PATHS
                ${s_wxRootDir}/lib
                DOC "Path to wxWidgets libraries?"
                NO_DEFAULT_PATH
            )
        else( MINGW AND NOT wxWidgets_MINGW_FROM_SOURCE )
            if( wxWidgets_USE_STATIC )
                find_path(wxWidgets_LIB_DIR
                    NAMES ${s_libPre}wxpng.${s_libExt} ${s_libPre}wxpngd.${s_libExt}
                    PATHS
                    ${s_wxRootDir}/lib/${s_wxCompiler}_lib  
                    DOC "Path to wxWidgets libraries?"
                    NO_DEFAULT_PATH
                )
            else( wxWidgets_USE_STATIC )
                find_path(wxWidgets_LIB_DIR
                    NAMES ${s_libPre}wxpng.${s_libExt} ${s_libPre}wxpngd.${s_libExt}
                    PATHS
                    ${s_wxRootDir}/lib/${s_wxCompiler}_dll   
                    DOC "Path to wxWidgets libraries?"
                    NO_DEFAULT_PATH
                )
            endif( wxWidgets_USE_STATIC )        
        endif( MINGW AND NOT wxWidgets_MINGW_FROM_SOURCE )
        mark_as_advanced( FORCE wxWidgets_LIB_DIR )
        if ( NOT wxWidgets_LIB_DIR )
                ERROR_MSG( FATAL_ERROR "wxWidgets_LIB_DIR for wxWidgets_USE_STATIC = ${wxWidgets_USE_STATIC} was not found")
        endif ( NOT wxWidgets_LIB_DIR )
        
        # the chosen configuration can now be tested
        if( NOT EXISTS ${wxWidgets_LIB_DIR}/${wxWidgets_CONFIGURATION} )
            # Search for possible configuration type availabilities
            foreach(CFG mswunivud mswunivd mswud mswd mswunivu mswuniv mswu msw)
                set(WX_${CFG}_FOUND FALSE)
                if(EXISTS ${wxWidgets_LIB_DIR}/${CFG})
                    set(s_wxConfiguration_LIST ${s_wxConfiguration_LIST} ${CFG})
                    set(WX_${CFG}_FOUND TRUE)
                endif()
            endforeach(CFG)                  
            set( wxWidgets_SELECTED_CONFIG wxWidgets_SELECTED_CONFIG-NOTFOUND CACHE STRING
                                "Selected wxWidgets configuration"  FORCE )                            
            ERROR_MSG( FATAL_ERROR "The configuration ${wxWidgets_LIB_DIR}/${wxWidgets_CONFIGURATION} was not found, we have ${s_wxConfiguration_LIST}")            
        else()    
            set( wxWidgets_SELECTED_CONFIG ${wxWidgets_LIB_DIR}/${wxWidgets_CONFIGURATION} CACHE STRING
                                "Selected wxWidgets configuration"  FORCE )                            
        endif()          

        # If wxWidgets_LIB_DIR changed, clear all libraries.
        if(NOT s_wxLibDir STREQUAL wxWidgets_LIB_DIR)
            set(s_wxLibDir ${wxWidgets_LIB_DIR} CACHE INTERNAL "wxWidgets_LIB_DIR")
            mark_as_advanced( FORCE s_wxLibDir )
            WX_CLEAR_ALL_DBG_LIBS()
            WX_CLEAR_ALL_REL_LIBS()
        endif()                
        
        set(wxWidgets_FOUND TRUE)
               
        if( wxWidgets_LIB_DIR MATCHES ".*[dD][lL][lL].*")
            dbg_msg("detected SHARED/DLL tree wxWidgets_LIB_DIR=${wxWidgets_LIB_DIR}")
            # add define for correct dllimport to link against WX DLL
            set(wxWidgets_DEFINITIONS ${wxWidgets_DEFINITIONS} "-DWXUSINGDLL")
        endif()
                  
        # if release config was selected, and both release/debug exist
        if (WX_${wxWidgets_CONFIGURATION}d_FOUND)
            # default to false, to not create string with optimized/debug is not possible in caller.
            option(wxWidgets_USE_REL_AND_DBG
            "Use release and debug configurations?" FALSE)
            set(WX_USE_REL_AND_DBG ${wxWidgets_USE_REL_AND_DBG})
        else (WX_${wxWidgets_CONFIGURATION}d_FOUND)
            # if the option exists, force it to false
            if(wxWidgets_USE_REL_AND_DBG)
                set(wxWidgets_USE_REL_AND_DBG FALSE CACHE BOOL
                "No ${wxWidgets_CONFIGURATION}d found." FORCE)
            endif(wxWidgets_USE_REL_AND_DBG)
            set(WX_USE_REL_AND_DBG FALSE)
        endif (WX_${wxWidgets_CONFIGURATION}d_FOUND)

        if( MINGW AND NOT wxWidgets_MINGW_FROM_SOURCE )           
            set( s_wxRootDir_wx_include ${wxWidgets_ROOT_DIR}/include/common )
            set( s_wxRootDir_contrib_include ${wxWidgets_ROOT_DIR}/include/common )
        else( MINGW AND NOT wxWidgets_MINGW_FROM_SOURCE )   
            set( s_wxRootDir_wx_include ${wxWidgets_ROOT_DIR}/include )
            set( s_wxRootDir_contrib_include ${wxWidgets_ROOT_DIR}/contrib/include )
        endif( MINGW AND NOT wxWidgets_MINGW_FROM_SOURCE )     
        
        # Set wxWidgets main include directory.
        if (EXISTS ${s_wxRootDir_wx_include}/wx/wx.h)
            set(wxWidgets_INCLUDE_DIRS ${s_wxRootDir_wx_include})
        else (EXISTS ${s_wxRootDir_wx_include}/wx/wx.h)
            ERROR_MSG( ERROR "WXWIDGET_FOUND FALSE because s_wxRootDir=${s_wxRootDir} has no ${s_wxRootDir}/include/wx/wx.h")
            set(wxWidgets_FOUND FALSE)
        endif (EXISTS ${s_wxRootDir_wx_include}/wx/wx.h)
            
        # Set wxWidgets lib setup include directory.
        if (EXISTS ${wxWidgets_LIB_DIR}/${wxWidgets_CONFIGURATION}/wx/setup.h)
            set(wxWidgets_INCLUDE_DIRS ${wxWidgets_LIB_DIR}/${wxWidgets_CONFIGURATION} ${wxWidgets_INCLUDE_DIRS} )
        else ()
            ERROR_MSG( ERROR "WXWIDGET_FOUND FALSE because  ${wxWidgets_LIB_DIR}/${wxWidgets_CONFIGURATION}/wx/setup.h does not exists.")
            set(wxWidgets_FOUND FALSE)
        endif ()

        find_build_variable( "WXVER_MAJOR" )                        
        if ( "A${wxWidgets_WXVER_MAJOR}" STRGREATER "A" )
            find_build_variable( "WXVER_MINOR" )            
            set( wxWidgets_PFVERSION "${wxWidgets_WXVER_MAJOR}${wxWidgets_WXVER_MINOR}" )
        endif ( "A${wxWidgets_WXVER_MAJOR}" STRGREATER "A" )
    
        find_build_variable( "USE_GDIPLUS" )                        
        if ( "${wxWidgets_USE_GDIPLUS}" STREQUAL "1" )
            list( APPEND wxWidgets_complibs gdiplus )
        endif ( "${wxWidgets_USE_GDIPLUS}" STREQUAL "1" )
        find_build_variable( "USE_OPENGL" )    
        if ( "${wxWidgets_USE_OPENGL}" STREQUAL "1" )
            set(wxWidgets_FIND_COMPONENTS ${wxWidgets_FIND_COMPONENTS} gl )
        endif ( "${wxWidgets_USE_OPENGL}" STREQUAL "1" )
                       
        if ( EXISTS ${s_wxRootDir_contrib_include} )
            set( wxWidgets_INCLUDE_DIRS ${wxWidgets_INCLUDE_DIRS} ${s_wxRootDir_contrib_include} )
        endif ( EXISTS ${s_wxRootDir_contrib_include} )
    
        # Find wxWidgets libraries.
        WX_FIND_LIBS("${wxWidgets_UNV}" "${wxWidgets_UCD}" "${wxWidgets_DBG}")
        if (WX_USE_REL_AND_DBG)
            WX_FIND_LIBS("${wxWidgets_UNV}" "${wxWidgets_UCD}" "d")
        endif (WX_USE_REL_AND_DBG)
                                       
        # Libraries we are interested in.
        if ( NOT wxWidgets_FIND_COMPONENTS )
            # Default minimal use setting (i.e., link to only core,base).
            set(wxWidgets_FIND_COMPONENTS ${wxWidgets_STD_LIBRARIES} )
        endif ( NOT wxWidgets_FIND_COMPONENTS )
        
        if (wxWidgets_FIND_COMPONENTS MATCHES std)
            # replace std by the list of STD libs
            LIST(APPEND wxWidgets_FIND_COMPONENTS ${wxWidgets_STD_LIBRARIES} )
            LIST(REMOVE_ITEM wxWidgets_FIND_COMPONENTS std)
            # TODO: check that "mono"  and base,core aren't added together
        endif (wxWidgets_FIND_COMPONENTS MATCHES std)
        
        if (wxWidgets_PFVERSION MATCHES "[0-2][0-8].*" )
        else()      
            list(FIND wxWidgets_FIND_COMPONENTS stc has-stc )
            if ( ${has-stc} EQUAL -1 )
                set(wxWidgets_FIND_COMPONENTS ${wxWidgets_FIND_COMPONENTS} stc )
            endif ()      
        endif ()      
    
        # Always add the common required libs.
        LIST(APPEND wxWidgets_FIND_COMPONENTS ${wxWidgets_COMMON_LIBRARIES} )
        
        # Settings for requested libs (i.e., include dir, libraries, etc.).
        WX_SET_LIBRARIES(wxWidgets_FIND_COMPONENTS "${wxWidgets_DBG}")
        
        set( wxWidgets_RC ${s_wxRootDir_wx_include}/wx/msw/wx.rc )
        # used in ADD_EXECUTABLE is the WIN32GUI to tell it is a typical windows GUI executable.
        set( WIN32GUI WIN32 )                
    endif (s_wxRootDir)

    set( wxWidgets_INCLUDE_DIRS ${wxWidgets_INCLUDE_DIRS} CACHE STRING  "" FORCE)
    set( wxWidgets_LIBRARIES ${wxWidgets_LIBRARIES} CACHE STRING  "" FORCE)
    set( wxWidgets_LIBRARY_DIRS ${wxWidgets_LIB_DIR} CACHE STRING  "" FORCE)
    set( wxWidgets_CXX_FLAGS ${wxWidgets_CXX_FLAGS} CACHE STRING  "" FORCE)
    set( wxWidgets_DEFINITIONS ${wxWidgets_DEFINITIONS} CACHE STRING  "" FORCE)
    
endif ( ${wxWidgets_FIND_STYLE} STREQUAL "WIN32_STYLE_FIND" )

#=====================================================================
# unix style find
#=====================================================================
if( ${wxWidgets_FIND_STYLE} STREQUAL "UNIX_STYLE_FIND" )

    if( wxWidgets_USE_MONOLITHIC )
        set(wxWidgets_FIND_COMPONENTS ${wxWidgets_FIND_COMPONENTS} mono stc )
    endif( wxWidgets_USE_MONOLITHIC )

    set( wxWidgets_SELECT_OPTIONS )
    
    # WASCANA needed this
    #set( wxWidgets_SELECT_OPTIONS "${wxWidgets_SELECT_OPTIONS} --prefix=/mingw")

    #-----------------------------------------------------------------
    # UNIX: Helper MACROS
    #-----------------------------------------------------------------

    # 
    # Set wxWidgets_SELECT_OPTIONS to wx-config options for selecting
    # among multiple builds.
    #
    macro(WX_CONFIG_SELECT_SET_OPTIONS)
      set(wxWidgets_SELECT_OPTIONS "")
      foreach(_opt_name debug static unicode universal)
        string(TOUPPER ${_opt_name} _upper_opt_name)
        if( DEFINED wxWidgets_USE_${_upper_opt_name} )
          if( wxWidgets_USE_${_upper_opt_name} )
            list(APPEND wxWidgets_SELECT_OPTIONS --${_opt_name}=yes)
          else( wxWidgets_USE_${_upper_opt_name} )
            list(APPEND wxWidgets_SELECT_OPTIONS --${_opt_name}=no)
          endif( wxWidgets_USE_${_upper_opt_name} )
        else( DEFINED wxWidgets_USE_${_upper_opt_name} )
            list(APPEND wxWidgets_SELECT_OPTIONS --${_opt_name}=no)
        endif( DEFINED wxWidgets_USE_${_upper_opt_name} )
      endforeach(_opt_name)
      list( APPEND wxWidgets_SELECT_OPTIONS --toolkit=${wxWidgets_PORT} )
      list( APPEND wxWidgets_SELECT_OPTIONS --version=${wxWidgets_PFVERSION} )
    endmacro(WX_CONFIG_SELECT_SET_OPTIONS)

    #-----------------------------------------------------------------
    # UNIX: Start actual work.
    #-----------------------------------------------------------------
    # Support cross-compiling, only search in the target platform.
    wx_find_root()
       
    if(wxWidgets_CONFIG_EXECUTABLE)

        # get defaults based on "wx-config --selected-config" only the first time
        if ( NOT DEFINED wxWidgets_DEFAULT_SELECTED_CONFIG )
            set( wxWidgets_SELECTED_CONFIG ${wxWidgets_DEFAULT_SELECTED_CONFIG} )
            wx_find_defaultconfig( ${wxWidgets_ROOT_DIR} )
        endif()

        # process selection to set wxWidgets_SELECT_OPTIONS
        # define the query string for wx-config, to test for the wanted wxWidgets configuration
        WX_CONFIG_SELECT_SET_OPTIONS()
        dbg_msg("wxWidgets_SELECT_OPTIONS=${wxWidgets_SELECT_OPTIONS}")

        # try to get the default wxWidgets configuration by query to wx-config
        set(wxWidgets_FOUND TRUE)
                                   
        # do the config test for all the wanted/found options to see if it is available.       
        execute_process(
            COMMAND sh "${wxWidgets_CONFIG_EXECUTABLE}" ${wxWidgets_SELECT_OPTIONS}  --selected-config          
            OUTPUT_VARIABLE wxWidgets_SELECTED_CONFIG_test
            RESULT_VARIABLE wxWidgets_WANTED_AVAILABLE
            ERROR_QUIET
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )   
                
        if ( wxWidgets_WANTED_AVAILABLE EQUAL 0 )
            # if this went oke, the extra information will be retrieved like flags paths prefix etc.
            set( wxWidgets_SELECTED_CONFIG ${wxWidgets_SELECTED_CONFIG_test} CACHE STRING
                                        "Selected wxWidgets configuration"  FORCE )        
          
            # run the wx-config program to get cxxflags
            execute_process(
                COMMAND sh "${wxWidgets_CONFIG_EXECUTABLE}" ${wxWidgets_SELECT_OPTIONS} --cppflags
                OUTPUT_VARIABLE wxWidgets_DEFINITIONS
                RESULT_VARIABLE RET
                ERROR_QUIET
                OUTPUT_STRIP_TRAILING_WHITESPACE        
            )   
            if( RET EQUAL 0 )
                #preprocessor flags, but also contained -I, skip them here.
                string( REGEX MATCHALL "-D.*[^ ;]+" wxWidgets_DEFINITIONS  ${wxWidgets_DEFINITIONS})           
                separate_arguments( wxWidgets_DEFINITIONS )
                dbg_msg("\nwxWidgets_DEFINITIONS=${wxWidgets_DEFINITIONS}")
            else( RET EQUAL 0 )
                ERROR_MSG( ERROR "${wxWidgets_CONFIG_EXECUTABLE} ${wxWidgets_SELECT_OPTIONS};--cppflags FAILED with RET=${RET}")
                set(wxWidgets_FOUND FALSE)
            endif( RET EQUAL 0 )

            # run the wx-config program to get cxxflags
            execute_process(
                COMMAND sh "${wxWidgets_CONFIG_EXECUTABLE}" ${wxWidgets_SELECT_OPTIONS} --cxxflags
                OUTPUT_VARIABLE wxWidgets_CXX_FLAGS
                RESULT_VARIABLE RET
                ERROR_QUIET
                OUTPUT_STRIP_TRAILING_WHITESPACE        
            )   
            if( RET EQUAL 0 )
                # parse definitions from cxxflags

                string( REGEX REPLACE "[\r\n]+$" "" wxWidgets_CXX_FLAGS  ${wxWidgets_CXX_FLAGS} )
                # drop -D* from CXXFLAGS
                # string(REGEX REPLACE "-D[^ ;]*" ""  wxWidgets_CXX_FLAGS  ${wxWidgets_CXX_FLAGS})
                #string(REGEX REPLACE ";-D[^;]+$" "" wxWidgets_CXX_FLAGS "${wxWidgets_CXX_FLAGS}")
                #string(REGEX REPLACE "^-D[^;]+$" "" wxWidgets_CXX_FLAGS "${wxWidgets_CXX_FLAGS}")            

                string(REGEX REPLACE "-D.*[^ ;]+" "" wxWidgets_CXX_FLAGS "${wxWidgets_CXX_FLAGS}")
                
                # parse incdirs from cxxflags, drop -I prefix
                string(REGEX MATCHALL "-I[^ ;]*" wxWidgets_INCLUDE_DIRS  ${wxWidgets_CXX_FLAGS})
                string(REGEX REPLACE "-I" ""  wxWidgets_INCLUDE_DIRS "${wxWidgets_INCLUDE_DIRS}")
                separate_arguments( wxWidgets_INCLUDE_DIRS )
                # drop -I* from CXXFLAGS
                string(REGEX REPLACE "-I[^ ;]*" ""  wxWidgets_CXX_FLAGS  ${wxWidgets_CXX_FLAGS})
                
                set( wxWidgets_CXX_FLAGS -Wformat=0 ${wxWidgets_CXX_FLAGS} )
                
            else( RET EQUAL 0 )
                ERROR_MSG( ERROR "${wxWidgets_CONFIG_EXECUTABLE} ${wxWidgets_SELECT_OPTIONS};--cxxflags FAILED with RET=${RET}")
                set(wxWidgets_FOUND FALSE)
            endif( RET EQUAL 0 )

            # run the wx-config program to get the libs
            # - NOTE: wx-config doesn't verify that the libs requested exist
            #         it just produces the names. Maybe a TRY_COMPILE would
            #         be useful here...
            #string(REPLACE ";" "," wxWidgets_FIND_COMPONENTS "${wxWidgets_FIND_COMPONENTS}")
            string(REGEX REPLACE ";" "," wxWidgets_FIND_COMPONENTS "${wxWidgets_FIND_COMPONENTS}")
            execute_process(
                COMMAND sh ${wxWidgets_CONFIG_EXECUTABLE} ${wxWidgets_SELECT_OPTIONS} --libs ${wxWidgets_FIND_COMPONENTS}
                OUTPUT_VARIABLE wxWidgets_LIBRARIES
                RESULT_VARIABLE RET
                ERROR_QUIET
                OUTPUT_STRIP_TRAILING_WHITESPACE
            )
            if( "${RET}" STREQUAL "0" )
                string(REGEX REPLACE " " ";" wxWidgets_LIBRARIES "${wxWidgets_LIBRARIES}")
                string(REGEX REPLACE "-framework;" "-framework " wxWidgets_LIBRARIES "${wxWidgets_LIBRARIES}")
                
                # extract linkdirs (-L) for rpath (i.e., LINK_DIRECTORIES)
                string(REGEX MATCHALL "-L[^ ;]*" wxWidgets_LIBRARY_DIRS  "${wxWidgets_LIBRARIES}")
                string(REGEX REPLACE "-L" ""  wxWidgets_LIBRARY_DIRS "${wxWidgets_LIBRARY_DIRS}")
                # convert space to semicolons for list
                string(REGEX REPLACE " " ";" wxWidgets_LIBRARY_DIRS "${wxWidgets_LIBRARY_DIRS}")

                string(REGEX REPLACE "-L[^ ;]*" ""  wxWidgets_LIBRARIES "${wxWidgets_LIBRARIES}")
                string(REGEX MATCHALL "-l[^ ;]*" wxWidgets_LIBRARIES_dynamic  "${wxWidgets_LIBRARIES}")
                string(REGEX MATCHALL "/[^ ;]*\\.a" wxWidgets_LIBRARIES_static  "${wxWidgets_LIBRARIES}")
                # first add static full path next dynamic and/or static using -l option 
                # ( options like -static -Bstatic -Bdynamic or ignored currently )
                # with -l option can be shared  or static, but shared is searched first.  
                # So a split should be made, but wx-config does not use this -Bstatic etc. flags.
                set( wxWidgets_LIBRARIES "${wxWidgets_LIBRARIES_static};${wxWidgets_LIBRARIES_dynamic}" )
                string(REGEX REPLACE "-l" ""  wxWidgets_LIBRARIES "${wxWidgets_LIBRARIES}")
            else( "${RET}" STREQUAL "0" )
                ERROR_MSG( ERROR "${wxWidgets_CONFIG_EXECUTABLE} ${wxWidgets_SELECT_OPTIONS} --libs ${wxWidgets_FIND_COMPONENTS} FAILED with RET=${RET}")
                set(wxWidgets_FOUND FALSE)
            endif( "${RET}" STREQUAL "0" )
          
            # get the path to the libraries and include files e.g /usr/local/
            execute_process(
                COMMAND sh ${wxWidgets_CONFIG_EXECUTABLE} ${wxWidgets_SELECT_OPTIONS} --prefix
                OUTPUT_VARIABLE wxWidgets_PREFIX
                RESULT_VARIABLE RET
                ERROR_QUIET
                OUTPUT_STRIP_TRAILING_WHITESPACE
            )
            string( REGEX REPLACE "[\r\n]+$" "" wxWidgets_PREFIX ${wxWidgets_PREFIX} )

            if( CYGWIN OR MINGW )
                get_filename_component(wxWidgets_ROOT_DIR ${wxWidgets_CONFIG_EXECUTABLE} PATH)
                set( wxWidgets_ROOT_DIR ${wxWidgets_ROOT_DIR}/.. )
                #set( wxWidgets_RC ${wxWidgets_ROOT_DIR}/include/wx/msw/wx.rc )
            else( CYGWIN OR MINGW )
                set( wxWidgets_RC "" )
            endif( CYGWIN OR MINGW )
            set( WIN32GUI "" )
          
            set(wxWidgets_LIB_PATH "${wxWidgets_PREFIX}/lib" )

        else ( wxWidgets_WANTED_AVAILABLE EQUAL 0 )
            set(wxWidgets_FOUND FALSE)            
            ERROR_MSG( ERROR "\n${wxWidgets_SELECT_OPTIONS} failed. \nThe default configuration found with: ${wxWidgets_CONFIG_EXECUTABLE} --selected_config \nis the following: ${wxWidgets_DEFAULT_SELECTED_CONFIG}\n" )
        endif ( wxWidgets_WANTED_AVAILABLE EQUAL 0 )
                    
    endif(wxWidgets_CONFIG_EXECUTABLE)
        
    # may have been changed, set again! 
    if( wxWidgets_USE_UNIVERSAL )
        set( wxWidgets_UNV "univ" )
    endif( wxWidgets_USE_UNIVERSAL )
    if ( wxWidgets_USE_UNICODE )
        set( wxWidgets_UCD "u" )
    endif ( wxWidgets_USE_UNICODE )        
    if( wxWidgets_USE_DEBUG )
        set( wxWidgets_DBG "d" )
    endif( wxWidgets_USE_DEBUG )        
    if( wxWidgets_USE_STATIC )
        set( WXLIBEXT ".a" )
    else( wxWidgets_USE_STATIC )
        set( WXLIBEXT ".so" )
    endif( wxWidgets_USE_STATIC ) 


endif( ${wxWidgets_FIND_STYLE} STREQUAL "UNIX_STYLE_FIND" )

#=====================================================================
# no style error
#=====================================================================
if( ${wxWidgets_FIND_STYLE} STREQUAL "NOT_DEFINED_STYLE_FIND" )
    if(NOT wxWidgets_FIND_QUIETLY)
        MESSAGE(STATUS "${CMAKE_CURRENT_LIST_FILE}(${CMAKE_CURRENT_LIST_LINE}): \n"
        "  Platform unknown/unsupported. It's neither WIN32 nor UNIX style find.")
    endif(NOT wxWidgets_FIND_QUIETLY)
endif( ${wxWidgets_FIND_STYLE} STREQUAL "NOT_DEFINED_STYLE_FIND" )

mark_as_advanced( FORCE wxWidgets_INCLUDE_DIRS )
mark_as_advanced( FORCE wxWidgets_LIBRARIES )
mark_as_advanced( FORCE wxWidgets_LIBRARY_DIRS )
mark_as_advanced( FORCE wxWidgets_CXX_FLAGS )
mark_as_advanced( FORCE wxWidgets_DEFINITIONS )

if (wxWidgets_FOUND)
    if(NOT wxWidgets_FIND_QUIETLY)
        MESSAGE( STATUS "Configuration from ${wxWidgets_ROOT_DIR}" )
        MESSAGE( STATUS "Configuration ${wxWidgets_CONFIG_EXECUTABLE} ${wxWidgets_SELECT_OPTIONS}" )
        MESSAGE( STATUS "wxWidgets_PFVERSION       : ${wxWidgets_PFVERSION}" )
        MESSAGE( STATUS "wxWidgets_USE_DEBUG       : ${wxWidgets_USE_DEBUG}" )
        MESSAGE( STATUS "wxWidgets_USE_UNICODE     : ${wxWidgets_USE_UNICODE}" )
        MESSAGE( STATUS "wxWidgets_USE_STATIC      : ${wxWidgets_USE_STATIC}" )
        MESSAGE( STATUS "wxWidgets_USE_UNIVERSAL   : ${wxWidgets_USE_UNIVERSAL}" )            
        MESSAGE( STATUS "wxWidgets_FOUND           : ${wxWidgets_FOUND}")
        MESSAGE( STATUS "wxWidgets_INCLUDE_DIRS    : ${wxWidgets_INCLUDE_DIRS}")
        MESSAGE( STATUS "wxWidgets_LIBRARY_DIRS    : ${wxWidgets_LIBRARY_DIRS}")
        MESSAGE( STATUS "wxWidgets_LIBRARIES       : ${wxWidgets_LIBRARIES}")
        MESSAGE( STATUS "wxWidgets_CXX_FLAGS       : ${wxWidgets_CXX_FLAGS}")
        MESSAGE( STATUS "wxWidgets_DEFINITIONS     : ${wxWidgets_DEFINITIONS}")            
        MESSAGE( STATUS "wxWidgets_FIND_COMPONENTS : ${wxWidgets_FIND_COMPONENTS}")
    endif(NOT wxWidgets_FIND_QUIETLY)                     
endif(wxWidgets_FOUND)   

# add convenience use file
if (wxWidgets_FOUND)
    # get dir of this file which may reside in
    # - CMAKE_MAKE_ROOT/Modules  on CMake installation
    # - CMAKE_MODULE_PATH if user prefers his own specialized version
    get_filename_component(wxWidgets_CURRENT_LIST_DIR ${CMAKE_CURRENT_LIST_FILE} PATH)
    set(wxWidgets_USE_FILE "${wxWidgets_CURRENT_LIST_DIR}/UsewxWidgets.cmake")
    # check
    if (NOT EXISTS ${wxWidgets_USE_FILE})
        if ( NOT  wxWidgets_FIND_QUIETLY)
            MESSAGE( "Your Find/Use wxWidgets installation is wrong. wxWidgets_USE_FILE=${wxWidgets_USE_FILE} not found.")
        endif(NOT  wxWidgets_FIND_QUIETLY)
    endif(NOT EXISTS ${wxWidgets_USE_FILE})    
else(wxWidgets_FOUND)
    # make FIND_PACKAGE friendly
    if(NOT wxWidgets_FIND_QUIETLY)
        if(wxWidgets_FIND_REQUIRED)
            MESSAGE(FATAL_ERROR
            "wxWidgets required, please specify it's location.")
        else(wxWidgets_FIND_REQUIRED)
            MESSAGE(STATUS "ERROR: wxWidgets was not found.")
        endif(wxWidgets_FIND_REQUIRED)
    endif(NOT wxWidgets_FIND_QUIETLY)    
endif(wxWidgets_FOUND)

dbg_msg("wxWidgets_FOUND           : ${wxWidgets_FOUND}")
dbg_msg("wxWidgets_INCLUDE_DIRS    : ${wxWidgets_INCLUDE_DIRS}")
dbg_msg("wxWidgets_LIBRARY_DIRS    : ${wxWidgets_LIBRARY_DIRS}")
dbg_msg("wxWidgets_DEFINITIONS     : ${wxWidgets_DEFINITIONS}")
dbg_msg("wxWidgets_CXX_FLAGS       : ${wxWidgets_CXX_FLAGS}")
dbg_msg("wxWidgets_LIBRARIES          : ${wxWidgets_LIBRARIES}")
dbg_msg("wxWidgets_USE_FILE        : ${wxWidgets_USE_FILE}")
dbg_msg("wxWidgets_FIND_COMPONENTS : ${wxWidgets_FIND_COMPONENTS}")



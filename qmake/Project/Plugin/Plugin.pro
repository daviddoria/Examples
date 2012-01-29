CONFIG      += designer plugin

TARGET      = $$qtLibraryTarget($$TARGET)

TEMPLATE    = lib

QTDIR_build:DESTDIR     = $$QT_BUILD_TREE/plugins/designer

INCLUDEPATH += $$PWD/../ # this will produce WARNING: Failure to find: Project.h. That is because qmake does not look in the include path, only the current directory

#HEADERS     = Project.h
HEADERS     = $$PWD/../Project.h

# install
target.path = $$[QT_INSTALL_PLUGINS]/designer
sources.files = $$SOURCES $$HEADERS *.pro
sources.path = $$[QT_INSTALL_EXAMPLES]/designer/LabeledSliderPlugin
INSTALLS += target sources

symbian: include($$QT_SOURCE_TREE/examples/symbianpkgrules.pri)

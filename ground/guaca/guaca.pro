#-------------------------------------------------
#
# Project created by QtCreator 2017-06-27T13:14:04
#
#-------------------------------------------------

QT       += core gui concurrent network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

QT += testlib

QMAKE_CXXFLAGS += -std=c++0x

TARGET = guaca
TEMPLATE = app

unix:!mac {
  target.path = /usr/local/bin
  desktopfile.path = /usr/share/applications
  desktopfile.files += guaca.desktop
  INSTALLS += desktopfile
  iconfile.path = /usr/share/icons/hicolor/scalable/apps
  iconfile.files = images/guaca.svg
  INSTALLS += iconfile
}

FLIGHT_DIR = ../..

INSTALLS += target

SOURCEPATH += $$FLIGHT_DIR/common/ \
              $$FLIGHT_DIR/external_libs/linklist/ \

INCLUDEPATH += $$FLIGHT_DIR/common/include/ \
               $$FLIGHT_DIR/external_libs/linklist/ \
               $$FLIGHT_DIR/liblinklist/

SOURCES += main.cpp\
        mainwindow.cpp \
        $$FLIGHT_DIR/liblinklist/linklist.c \
        $$FLIGHT_DIR/liblinklist/linklist_compress.c \
        $$FLIGHT_DIR/liblinklist/linklist_writer.c \
        $$FLIGHT_DIR/liblinklist/linklist_connect.c \
        options.cpp \
        logscroll.cpp \
        server.cpp

HEADERS  += mainwindow.h \
        $$FLIGHT_DIR/liblinklist/linklist.h \
        $$FLIGHT_DIR/liblinklist/linklist_compress.h \
        $$FLIGHT_DIR/liblinklist/linklist_writer.h \
        $$FLIGHT_DIR/liblinklist/linklist_connect.h \
        options.h \
        logscroll.h \
        server.h

message($$SOURCES)
message($$HEADERS)

LIBS += -lssl -lcrypto


FORMS    += mainwindow.ui \
    options.ui \
    logscroll.ui

RESOURCES += \
    guacapics.qrc

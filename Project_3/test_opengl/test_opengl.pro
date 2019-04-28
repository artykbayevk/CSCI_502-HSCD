#-------------------------------------------------
#
# Project created by QtCreator 2016-11-16T22:48:02
#
#-------------------------------------------------

CONFIG   += thread
QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = test_opengl
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    myglwidget.cpp \
    mythread.cpp

HEADERS  += mainwindow.h \
    myglwidget.h \
    mythread.h

FORMS    += mainwindow.ui

## MAKE SURE THAT THE QT is same version, or change version here
#LIBS += -LC:/Qt/5.5/mingw492_32/lib/libQt5OpenGL.a -lopengl32


######################################################################
# Automatically generated by qmake (2.01a) Sun Jan 22 10:37:32 2012
######################################################################

TEMPLATE = app
TARGET =
DEPENDPATH += .../include
INCLUDEPATH += ../include

# Input
QT += opengl
HEADERS = DemoBase.h \
  Demo1.h \
  ../include/Rigid2D.h \
  ForceFunctions.h \
  DemosFramework.h

SOURCES = examples.cpp \
  DemosFramework.cpp \
  DemoBase.cpp \
  Demo1.cpp \
  ForceFunctions.cpp \
  ../include/Objects/RigidBody.cpp \
  ../include/Objects/RigidBodySystem.cpp \
  ../include/Objects/Force.cpp \
  ../include/Common/RigidException.cpp \
  ../include/Common/MathUtils.cpp \
  ../include/Common/feq.cpp \

QMAKE_CXXFLAGS += -std=c++0x -ggdb
QMAKE_CLEAN += examples *.o

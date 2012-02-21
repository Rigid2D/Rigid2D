######################################################################
# Automatically generated by qmake (2.01a) Sun Jan 22 10:37:32 2012
######################################################################

TEMPLATE = app
TARGET =
DEPENDPATH += .../include
INCLUDEPATH += ../include

# Input
QT += opengl
HEADERS = SampleDemo.h \
  ../include/Rigid2D.h \
  ForceFunctions.h \

SOURCES = examples.cpp \
  SampleDemo.cpp \
  ForceFunctions.cpp \
  ../include/Objects/RigidBody.cpp \
  ../include/Objects/RigidBodySystem.cpp \
	../include/Common/OdeRungeKutta4.cpp \
	../include/Common/RungeKutta4RigidBodySolver.cpp \
	../include/Common/RigidException.cpp \
  ../include/Objects/Force.cpp \
	../include/Common/MathUtils.cpp \
	../include/Common/feq.cpp

QMAKE_CXXFLAGS += -std=c++0x
QMAKE_CLEAN += examples *.o

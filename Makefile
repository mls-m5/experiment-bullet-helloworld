# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

# Default target executed when no arguments are given to make.
default_target: all
.PHONY : default_target

cmake_force:
.PHONY : cmake_force

OBJECTS = HelloWorld.o
libs = -lGL -lGLU -lglut -lSDL 
#/usr/local/lib/libBulletDynamics.a /usr/local/lib/libBulletCollision.a /usr/local/lib/libLinearMath.a
libs += -lLinearMath -lBulletDynamics -lBulletCollision
CPPFLAGS += -std=c++0x -g
CPPFLAGS += -I/usr/include/GL -I/usr/local/include/bullet/ -I/usr/include/bullet/
CPPFLAGS += -D_GNU_SOURCE=1 -D_REENTRANT -I/usr/include/SDL

all: $(OBJECTS)
	g++ HelloWorld.o -o HelloWorld  $(libs)


%.o: %.h
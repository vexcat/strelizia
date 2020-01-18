################################################################################
######################### User configurable parameters #########################
# filename extensions
CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp c++ cc

# probably shouldn't modify these, but you may need them below
ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR=$(ROOT)/bin
SRCDIR=$(ROOT)/src
INCDIR=$(ROOT)/include $(ROOT)/include/kernel $(ROOT)/src/tabu $(ROOT)/src/uzlib $(ROOT)/src/minibsdiff

WARNFLAGS+=
EXTRA_CFLAGS=
EXTRA_CXXFLAGS=-Winvalid-pch -Wall -pedantic -Wno-psabi -Wno-unused-function -Wno-sign-compare

# Set to 1 to enable hot/cold linking
USE_PACKAGE:=1

# Set this to 1 to add additional rules to compile your project as a PROS library template
IS_LIBRARY:=1
# TODO: CHANGE THIS!
LIBNAME:=libhot
VERSION:=1.0.0
# EXCLUDE_SRC_FROM_LIB= $(SRCDIR)/unpublishedfile.c
# this line excludes opcontrol.c and similar files
USER_SRC_FILES = $(shell find $(SRCDIR)/usr/ -type f \( -name '*.c' -o -name '*.cpp' \) )
EXCLUDE_SRC_FROM_LIB+=$(wildcard $(SRCDIR)/user/*.c) $(wildcard $(SRCDIR)/user/*.cpp)

# files that get distributed to every user (beyond your source archive) - add
# whatever files you want here. This line is configured to add all header files
# that are in the the include directory get exported
TEMPLATE_FILES=$(INCDIR)/**/*.h $(INCDIR)/**/*.hpp

.DEFAULT_GOAL=quick

################################################################################
################################################################################
########## Nothing below this line should be edited by typical users ###########
-include ./common.mk

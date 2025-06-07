###############################################################
#
# Purpose: Makefile for "webots_tcp_server" (Corrected Version)
# Author.: robotis
# Version: 0.2
# License: GPL
#
###############################################################

TARGET = webots_tcp_server
CXX = g++

# --- Compiler Settings ---
# All include paths are defined here.
INCLUDE_DIRS = -I/home/darwin/onnxruntime/include -I../../include -I../../../Framework/include

# All compiler flags are defined here.
CXXFLAGS = -O2 -DLINUX -g -Wall -fmessage-length=0 $(INCLUDE_DIRS)


# --- Linker Settings ---
# All library paths and libraries are defined here.
# The "-Wl,-rpath" part is a best practice that tells your program where
# to find the onnxruntime library when you run it.
LDFLAGS = -L/home/darwin/onnxruntime/build/Linux/Release -Wl,-rpath=/home/darwin/onnxruntime/build/Linux/Release
LIBS = -lonnxruntime -lpthread -lncurses -lrt -ljpeg


# --- Source Files ---
OBJS = webots_tcp_server.o


# --- Build Rules ---

# The default target
all: $(TARGET)

# Explicit rule to build a .o file from a .cpp file.
# This ensures our CXXFLAGS are always used.
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# Rule to link the final executable.
# Notice this now includes $(LDFLAGS) to correctly link the ONNX library.
$(TARGET): $(OBJS) ../../lib/darwin.a
	$(CXX) -o $(TARGET) $(OBJS) ../../lib/darwin.a $(LDFLAGS) $(LIBS)

# Rule to build the darwin.a dependency
darwin.a:
	make -C ../../build

# --- Cleaning Rules ---

clean:
	rm -f *.o $(TARGET) core *~ *.so *.lo

libclean:
	make -C ../../build clean

distclean: clean libclean
###############################################################
#
# Purpose: Makefile for "webots_tcp_server"
# Author.: robotis
# Version: 0.1
# License: GPL
#
###############################################################

TARGET =  webots_tcp_server

CXX = g++
INCLUDE_DIRS = -I../../include -I../../../Framework/include
ONNX_INCLUDE_PATH = -I/home/darwin/onnxruntime/include
ONNX_LIB_PATH = -L/home/darwin/onnxruntime/build/Linux/Release
ONNX_LIBS = -lonnxruntime

CXXFLAGS += $(ONNX_INCLUDE_PATH)
LDFLAGS += $(ONNX_LIB_PATH) $(ONNX_LIBS)
CXXFLAGS +=	-O2 -DLINUX -g -Wall -fmessage-length=0 $(INCLUDE_DIRS)
LIBS += -lpthread -lncurses -lrt -ljpeg

OBJS =	./webots_tcp_server.o


all: darwin.a $(TARGET)

clean:
	rm -f *.a *.o $(TARGET) core *~ *.so *.lo

libclean:
	make -C ../../build clean

distclean: clean libclean

darwin.a:
	make -C ../../build

$(TARGET): $(OBJS) ../../lib/darwin.a
	$(CXX) -o $(TARGET) $(OBJS) ../../lib/darwin.a $(LIBS)

# useful to make a backup "make tgz"
tgz: clean
	mkdir -p backups
	tar czvf ./backups/webots_tcp_server_`date +"%Y_%m_%d_%H.%M.%S"`.tgz --exclude backups *

###############################################################
#
# Purpose: Makefile for "rx_tx_serial_test"
# Author.: robotis
# Version: 0.1
# License: GPL
#
###############################################################

TARGET = rx_tx_serial_test

INCLUDE_DIRS = -I../../../include -I../../../../Framework/include

CXX = g++
CXXFLAGS += -O2 -DLINUX -Wall $(INCLUDE_DIRS)
#CXXFLAGS += -O2 -DDEBUG -DLINUX -Wall $(INCLUDE_DIRS)
LFLAGS += -lpthread -ljpeg -lrt

OBJECTS =   main.o

all: $(TARGET)

clean:
	rm -f *.a *.o $(TARGET) core *~ *.so *.lo

libclean:
	make -C ../../../build clean

distclean: clean libclean

darwin.a:
	make -C ../../../build

$(TARGET): darwin.a $(OBJECTS)
	$(CXX) $(CFLAGS) $(OBJECTS) ../../../lib/darwin.a -o $(TARGET) $(LFLAGS)
	chmod 755 $(TARGET)

# useful to make a backup "make tgz"
tgz: clean
	mkdir -p backups
	tar czvf ./backups/rx_tx_serial_test_`date +"%Y_%m_%d_%H.%M.%S"`.tgz --exclude backups *

TARGET = makeip

VERSION = 2.0.0

OBJECTS = utils.o vector.o crc.o mr.o field.o ip.o main.o

CC = gcc
STRIP = strip

CFLAGS = -O2 -Wall -DMAKEIP_VERSION=\"$(VERSION)\" -I/usr/local/include
LDFLAGS = -L/usr/local/lib -lpng -lz

INSTALLDIR = ..

EXECUTABLEEXTENSION =
ifeq ($(shell echo $(OS)),Windows_NT)
  EXECUTABLEEXTENSION := .exe
endif

OUTPUT = $(TARGET)$(EXECUTABLEEXTENSION)

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) -o $(OUTPUT) $(CFLAGS) $(OBJECTS) $(LDFLAGS)
	$(STRIP) $(OUTPUT)

install:
	mkdir -p $(INSTALLDIR)
	mv $(OUTPUT) $(INSTALLDIR)

.PHONY: clean distclean
clean:
	-rm -f *.o

distclean: clean
	-rm -f $(OUTPUT)

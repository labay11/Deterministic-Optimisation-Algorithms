RCODIR=$(shell pwd)
IDIR =./src
CC=gcc
CFLAGS=-I$(IDIR) -Wall -O3

ODIR=./build
LDIR =./src

LIBS=-lm

dirs:
	mkdir -p $(ODIR)

cgd: dirs
	$(CC) $(CFLAGS) $(LDIR)/ros.c $(LDIR)/cgd.c -o $(ODIR)/cgd.o $(LIBS)

lm: dirs
	$(CC) $(CFLAGS) $(LDIR)/ros.c $(LDIR)/lm.c -o $(ODIR)/lm.o $(LIBS)

all: cgd lm

run: all
	$(ODIR)/cgd.o
	$(ODIR)/lm.o

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ $(INCDIR)/*~ $(LDIR)/*.h.gch $(IDIR)/*.h.gch

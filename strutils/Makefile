
CC = gcc
AR = ar
LIB = strutils
CFLAGS = -Wall -O2

.PHONY: all static shared clean

all: static shared

bstrutils.o: bstrutils.c strutils.h
	$(CC) $(CFLAGS) -c bstrutils.c -o bstrutils.o

lib$(LIB).a: bstrutils.o
	$(AR) rcs $@ $^

main_static: main.c lib$(LIB).a
	$(CC) $(CFLAGS) main.c lib$(LIB).a -o main_static

static: lib$(LIB).a main_static

bstrutils_pic.o: bstrutils.c strutils.h
	$(CC) $(CFLAGS) -fPIC -c bstrutils.c -o bstrutils_pic.o

lib$(LIB).so: bstrutils_pic.o
	$(CC) -shared -o $@ $^

main_shared: main.c lib$(LIB).so
	$(CC) $(CFLAGS) main.c -L. -Wl,-rpath=. -l$(LIB) -o main_shared

shared: lib$(LIB).so main_shared

clean:
	rm -f *.o *.a *.so main_static main_shared
CC=g++
CFLAGS= -I/home/aboud/reactphysics3d/src -g	-Wall
LDFLAGS = -L/usr/local/lib64 -lreactphysics3d

default: fluidtrack

obj/fluidtrack.o: src/fluidtrack.cpp
	$(CC) -c src/fluidtrack.cpp -o obj/fluidtrack.o $(CFLAGS)

fluidtrack: obj/fluidtrack.o
	$(CC) obj/fluidtrack.o -o fluidtrack $(LDFLAGS)

clean:
	rm obj/*
	rm fluidtrack

TARGET = fluidtrack
CC = g++

DEBUGW = -Wall
DEBUG = -g -O0 $(DEBUGW)

CFLAGS = $(DEBUG)
LDFLAGS = $(DEBUG) -L/usr/local/lib64 -lreactphysics3d

PLATFORM = LINUX
SOURCE = src
INCLUDE = -Isrc -Ipicojson -I/home/aboud/reactphysics3d/src

OBJECTS = $(patsubst %.cpp, %.o, $(wildcard $(SOURCE)/*/*.cpp $(SOURCE)/*.cpp))
#HEADERS = $(wildcard $(SOURCE)/*/*.h $(SOURCE)/*.h)

default: $(TARGET)
all: default

debug:
	@echo "OBJECTS"
	@echo $(OBJECTS)
	@echo "HEADERS"
	@echo $(HEADERS)

%.o: %.cpp #$(HEADERS)
	$(CC) $(INCLUDE) -c $< -o $@ -D__$(PLATFORM)__ $(CFLAGS)

$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LDFLAGS)

clean:
	rm -f $(SOURCE)/*.o
	rm -f $(TARGET)

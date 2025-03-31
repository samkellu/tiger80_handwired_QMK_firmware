CXX = gcc
CXXFLAGS = -std=c++11 -g
SRC = emulator/emulator.c
TARGET = $(SRC:.c=)
SDLFLAGS = -lSDL2 -lm
INCL = bongo80/bongo80.c

all:build

build:
	$(CXX) $(SRC) $(INCL) -DUSE_EMULATOR $(SDLFLAGS) -o $(TARGET)

clean:
	rm -f $(TARGET)

run:
	./$(TARGET)
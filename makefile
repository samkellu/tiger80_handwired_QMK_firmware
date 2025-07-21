CXX = gcc
CXXFLAGS = -std=c++11 -g
SRC = emulator/emulator.c
TARGET = $(SRC:.c=)
SDLFLAGS = -lSDL2 -lm
INCL = doompad/doom.c doompad/bongo.c

all:build

build:
	$(CXX) $(SRC) $(INCL) -DUSE_EMULATOR $(SDLFLAGS) -o $(TARGET)

topdown:
	$(CXX) $(SRC) $(INCL) -DUSE_EMULATOR -DRENDER_DEBUG $(SDLFLAGS) -o $(TARGET)

clean:
	rm -f $(TARGET)

run:
	./$(TARGET)
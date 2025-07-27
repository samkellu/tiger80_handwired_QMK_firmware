CXX = gcc
CXXFLAGS = -g
SRC = emulator/emulator.c
TARGET = $(SRC:.c=)
SDLFLAGS = -lSDL2 -lm
INCL = shared/doom.c shared/bongo.c

all:build

build:
	$(CXX) $(SRC) $(INCL) -DUSE_EMULATOR $(SDLFLAGS) -o $(TARGET)

topdown:
	$(CXX) -g $(SRC) $(INCL) -DUSE_EMULATOR -DRENDER_DEBUG $(SDLFLAGS) -o $(TARGET)

clean:
	rm -f $(TARGET)

run:
	./$(TARGET)4
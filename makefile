CXX = g++
CXXFLAGS = -Wall -std=c++11 -g
SRC = scanline.cpp
TARGET = $(SRC:.cpp=)
SDLFLAGS = -lSDL2

all:build

build:
	$(CXX) $(SRC) $(SDLFLAGS) -o $(TARGET)

clean:
	rm -f $(TARGET)

run:
	./$(TARGET)
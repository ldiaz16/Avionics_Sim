CXX      ?= g++
CXXFLAGS  = -std=c++17 -Wall -Wextra -Wpedantic -O2

all: flight_sim

flight_sim: main.cpp flight_computer.cpp flight_computer.h
	$(CXX) $(CXXFLAGS) -o $@ main.cpp flight_computer.cpp

test_runner: tests.cpp flight_computer.cpp flight_computer.h
	$(CXX) $(CXXFLAGS) -o $@ tests.cpp flight_computer.cpp

run: flight_sim
	./flight_sim

test: test_runner
	./test_runner

clean:
	rm -f flight_sim test_runner

.PHONY: all run test clean

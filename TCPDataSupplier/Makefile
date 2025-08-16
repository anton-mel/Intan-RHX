CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2
LDFLAGS = -pthread

SHM_TARGET = shm_synthetic_supplier
SHM_SOURCES = shm_synthetic_supplier.cpp
SHM_OBJECTS = $(SHM_SOURCES:.cpp=.o)

TCP_TARGET = tcp_synthetic_server
TCP_SOURCES = tcp_synthetic_server.cpp
TCP_OBJECTS = $(TCP_SOURCES:.cpp=.o)

.PHONY: all clean run run-shm run-tcp

all: $(SHM_TARGET)

$(SHM_TARGET): $(SHM_OBJECTS)
	$(CXX) $(SHM_OBJECTS) -o $(SHM_TARGET) $(LDFLAGS)

$(TCP_TARGET): $(TCP_OBJECTS)
	$(CXX) $(TCP_OBJECTS) -o $(TCP_TARGET) $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(SHM_OBJECTS) $(TCP_OBJECTS) $(SHM_TARGET) $(TCP_TARGET)

run: $(SHM_TARGET)
	./$(SHM_TARGET)

run-shm: $(SHM_TARGET)
	./$(SHM_TARGET)

run-tcp: $(TCP_TARGET)
	./$(TCP_TARGET)



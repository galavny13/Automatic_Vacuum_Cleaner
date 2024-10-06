# Define the compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall -DUSE_GUI=0 -Icommon -Isimulator -Ialgorithm -Isimulation -fPIC
LDFLAGS = -ldl -pthread

# Define the executable name
TARGET = myrobot

# Define the source and header files
SRCS = simulation/simulator.cpp \
       simulation/House.cpp \
       simulation/VacuumCleaner.cpp \
       simulation/ConcreteWallsSensor.cpp \
       simulation/ConcreteDirtSensor.cpp \
       simulation/ConcreteBatteryMeter.cpp \
       bonus/SystemManager.cpp \
       simulator/AlgorithmRegistrar.cpp \
       simulation/utils.cpp \
       algo/MyAlgo.cpp \
       main.cpp

HDRS = common/AbstractAlgorithm.h \
       common/BatteryMeter.h \
       common/DirtSensor.h \
       common/enums.h \
       common/WallsSensor.h \
       simulation/simulator.h \
       simulation/House.h \
       simulation/VacuumCleaner.h \
       algo/MyAlgo.h \
       algo/algo1/algo_323855288_209648815_A.h \
       algo/algo2/algo_323855288_209648815_B.h \
       simulation/ConcreteWallsSensor.h \
       simulation/ConcreteDirtSensor.h \
       simulation/ConcreteBatteryMeter.h \
       bonus/button.h \
       bonus/SystemManager.h \
       common/AlgorithmRegistrar.h \
       algo/AlgorithmRegistration.h \
       simulation/utils.h

# Define the object files
OBJS = $(SRCS:.cpp=.o)

# Find all algorithm source files
ALGO_SRCS = $(wildcard algo/algo*/*.cpp)
ALGO_LIBS = $(ALGO_SRCS:.cpp=.so)

# Default target
all: $(TARGET) $(ALGO_LIBS)

# Link the executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS) $(LDFLAGS)

# Compile the source files into object files
%.o: %.cpp $(HDRS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Create shared libraries for algorithms
algo/algo%/%.so: algo/algo%/%.cpp algo/MyAlgo.cpp
	$(CXX) $(CXXFLAGS) -shared -o $@ $^ $(LDFLAGS)

# Clean up build files
clean:
	rm -f $(TARGET) $(OBJS) $(ALGO_LIBS)

# PHONY targets
.PHONY: all clean
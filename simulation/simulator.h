#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <typeinfo>
#include <cxxabi.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <regex>
#include <memory>
#include <utility>
#include <cstdio>
#include <optional>
#include "../common/AbstractAlgorithm.h"
#include "ConcreteBatteryMeter.h"
#include "ConcreteDirtSensor.h"
#include "ConcreteWallsSensor.h"
#include "VacuumCleaner.h"
#include "House.h"
#include "../algo/MyAlgo.h"
#include "../bonus/SystemManager.h"
#include "utils.h"

class MySimulator {
public:
    // Sets the cleaning algorithm for the simulation.
    void setAlgorithm(AbstractAlgorithm& algo, bool _summaryOnly, bool _animationMode,
                      std::vector<std::vector<int>> _houseMap, std::string _currentHouseName,
                      std::string _houseName, int _maxSteps, int _maxBatterySteps,
                      int _rows, int _cols, std::pair<int, int> _dockingStation, int _initialDirt);

    void run(std::atomic<bool>& timeoutFlag);

    void run();

    int getScore() const;

private:
    std::optional<SystemManager> systemManager;  // Use std::optional for delayed initialization

    std::shared_ptr<House> house; // Shared pointer to the House object
    std::shared_ptr<VacuumCleaner> vacuumCleaner; // Shared pointer to the VacuumCleaner object
    std::shared_ptr<AbstractAlgorithm> algorithm; // Shared pointer to the AbstractAlgorithm object
    std::shared_ptr<MyAlgo> myAlgo;

    std::unique_ptr<ConcreteWallsSensor> wallsSensor; // Unique pointer to the walls sensor
    std::unique_ptr<ConcreteDirtSensor> dirtSensor; // Unique pointer to the dirt sensor
    std::unique_ptr<ConcreteBatteryMeter> batteryMeter; // Unique pointer to the battery meter

    int maxSteps, maxBatterySteps, rows, cols; // Simulation parameters
    std::vector<std::vector<int>> houseMap; // 2D vector representing the house layout
    std::pair<int, int> dockingStation; // Coordinates of the docking station
    std::shared_ptr<std::vector<Step>> steps; // Vector to store the steps taken by the vacuum cleaner
    std::shared_ptr<bool> finished;
    std::shared_ptr<int> score;
    std::string houseName; // Name / description of the house in the input file
    std::string currentHouseName;
    bool summaryOnly;
    bool animationMode;
    int initialDirt;

    // ----------------------------------------------------- BONUS ----------------------------------------------------

    std::shared_ptr<bool> useGUI; // Flag to indicate whether GUI is available and should be used
};

#endif // SIMULATOR_H

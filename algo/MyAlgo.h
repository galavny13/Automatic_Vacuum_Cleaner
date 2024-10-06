#ifndef MY_ALGO_H
#define MY_ALGO_H

#include "common/AbstractAlgorithm.h"
#include "common/WallSensor.h"
#include "common/DirtSensor.h"
#include "common/BatteryMeter.h"
#include "common/enums.h"
#include "algorithm/AlgorithmRegistration.h"
#include <memory>
#include <map>
#include <vector>
#include <queue>
#include <cmath>
#include <set>
#include <utility>
#include <stdexcept>
#include <iostream>
#include <sstream>

// Only include House.h if it exists - this is to make sure we can run the algorithm also with other projects
#if __has_include("simulation/House.h")
#include "simulation/House.h"
#endif

class MyAlgo : public AbstractAlgorithm {
public:
    MyAlgo(); // Constructor for MyAlgorithm

    // This is now a non-virtual public method
    Step nextStep() override;

    void setMaxSteps(size_t maxSteps) override; // Set the maximum number of steps allowed for the simulation

    void setWallsSensor(const WallsSensor& wallsSensor) override; // Set the walls sensor to detect walls around the vacuum cleaner

    void setDirtSensor(const DirtSensor& dirtSensor) override; // Set the dirt sensor to detect dirt levels in the current cell

    void setBatteryMeter(const BatteryMeter& batteryMeter) override; // Set the battery meter to check the current battery state



#if __has_include("simulation/House.h")
    // NOTE - These 2 methods are only public because we use them in the VacuumCleaner class to avoid duplicating code.
    static void move(Step step, std::pair<int, int>& position, double& currentBatteryLevel,
                     int maxBatterySteps, int& steps,
                     std::shared_ptr<House> house = nullptr);

    static void moveBack(Step direction, double previousBatteryLevel, std::pair<int, int>& position, double& currentBatteryLevel,
                         int& steps, std::shared_ptr<House> house = nullptr);
#else
    static void move(Step step, std::pair<int, int>& position, double& currentBatteryLevel,
                     int maxBatterySteps, int& steps);

    static void moveBack(Step direction, double previousBatteryLevel, std::pair<int, int>& position, double& currentBatteryLevel,
                         int& steps);
#endif

    // ----------------------------------------------------- BONUS ----------------------------------------------------

    struct StepState {
        double previousBatteryLevel;
        std::map<std::pair<int, int>, bool> cleanablePositions;
        std::map<std::pair<int, int>, int> distanceToDock;
        bool inExplorationMode;
        bool exploredNewCell;
        bool exploredNewNorthernWall;
        bool exploredNewEasternWall;
        bool exploredNewSouthernWall;
        bool exploredNewWesternWall;
    };

    void enableSimulationMode();

    // Update the house map with current position information. NOTE - This is only public because we use it in our bonus
    // part of the code in SystemManager.cpp, since this helps us get a smoother animation.
    void updateHouseMap(const std::pair<int, int>& pos);

    // NOTE - This is only public because we use it in our bonus part of the code in SystemManager.cpp.
    [[nodiscard]] static std::pair<int, int> getNeighborPosition(const std::pair<int, int>& pos, Direction direction); // Get the neighbor position based on a direction

    // NOTE - This is only public because we use it in our bonus part of the code in SystemManager.cpp.
    void updateExplorationMetrics(); // Update exploration metrics

    // Getters. NOTE - These are only used as part of the bonus part of the project, in order to make sure the algorithm
    // can run smoothly on other projects.

    std::map<std::pair<int, int>, int> &getHouseMap();

    std::vector<Step> &getCleaningPath();

    int& getTotalDirt();

    std::pair<int, int> &getCurrentPosition();

    double& getCurrentBatteryLevel();

    int& getSteps();

    std::vector<StepState> &getStateHistory();

    std::map<std::pair<int, int>, bool> &getCleanablePositions();

    std::map<std::pair<int, int>, int> &getDistanceToDock();

    bool& getInExplorationMode();

    bool& getExploredNewWesternWall();

    bool& getExploredNewSouthernWall();

    bool& getExploredNewEasternWall();

    bool& getExploredNewNorthernWall();

    bool& getExploredNewCell();

protected:
    std::map<std::pair<int, int>, int> houseMap; // Map to store the house layout and dirt levels
    std::map<std::pair<int, int>, bool> cleanablePositions; // Map to store whether each position in the house is cleanable or not
    std::map<std::pair<int, int>, int> distanceToDock; // Stores shortest known distance to dock for each position
    std::vector<Step> cleaningPath; // Vector to store the planned cleaning path
    const std::pair<int, int> home{0, 0}; // Home (docking station) position
    bool inExplorationMode; // Flag to indicate if the vacuum is in exploration mode
    size_t maxSteps; // Maximum number of steps allowed in the simulation
    int totalDirt; // Total dirt in the mapped house
    int unexploredNeighbors; // Number of unexplored neighboring positions

    // Sensors
    const WallsSensor* wallsSensor; // Wall sensor object
    const DirtSensor* dirtSensor; // Dirt sensor object
    const BatteryMeter* batteryMeter; // Battery meter object

    // Represents the vacuum cleaner state
    std::pair<int, int> currentPosition; // Current position of the vacuum cleaner
    int maxBattery; // Maximum battery level
    double currentBatteryLevel; // Current battery level
    int steps; // Number of steps taken

    Step executeNextStep();

    [[nodiscard]] std::pair<bool, bool> shouldReturnToDock(const std::pair<int, int>& currentPos, double currentBattery) const; // Check if the vacuum should return to the docking station

    [[nodiscard]] std::pair<bool, bool> shouldFinish(const std::pair<int, int>& currentPos, size_t remainingSteps) const; // Check if the algorithm should finish

    Step moveTowardsDock(const std::pair<int, int>& currentPos); // Move towards the docking station

    [[nodiscard]] bool canExploreMore(const std::pair<int, int>& currentPos, int availableBattery, int availableSteps) const;

    [[nodiscard]] bool isExplorationComplete() const; // Check if exploration is complete

    [[nodiscard]] bool isHouseClean() const; // Check if the house is clean

    [[nodiscard]] std::vector<Step> findPath(const std::pair<int, int>& start, const std::pair<int, int>& goal) const; // Find a path between two positions

    void updateDistanceIfShorter(const std::pair<int, int>& pos, int newDistance);

    [[nodiscard]] static std::vector<std::pair<int, int>> getNeighbors(const std::pair<int, int>& pos); // Get neighboring positions

    [[nodiscard]] static Direction indexToDirection(int index); // Convert an index to a direction

    [[nodiscard]] static Step vectorToStep(const std::pair<int, int>& dir); // Convert a vector to a step

    static void charge(double& currentBatteryLevel, int maxBatterySteps); // Charge the vacuum cleaner at the docking station

    static Step reverseStep(Step step);


    // From MyAlgorithm

    virtual Step exploreNextStep(const std::pair<int, int>& currentPos); // Determine the next step during exploration

    void planCleaningPath(); // Plan the cleaning path

    virtual Step followCleaningPath(const std::pair<int, int>& currentPos); // Follow the planned cleaning path

    std::pair<int, int> findBestNextPosition(const std::pair<int, int>& currentPos, const std::set<std::pair<int, int>>& dirtyPositions,
                                             int remainingBattery, const std::map<std::pair<int, int>, int>& dirtChanges); // Find the best next position to clean

    bool shouldSwitchMode(double x, double y); // Determine if the vacuum should switch modes

    virtual bool shouldSwitchMode() = 0; // The actual shouldSwitchMode method will be implemented in the derived classes

    // ----------------------------------------------------- BONUS ----------------------------------------------------

    bool simulationMode = false;

    bool exploredNewCell;
    bool exploredNewNorthernWall;
    bool exploredNewEasternWall;
    bool exploredNewSouthernWall;
    bool exploredNewWesternWall;

    std::vector<StepState> stateHistory; // Use deque as a circular buffer

    StepState currentState;

    void updateStateHistory();
};

#endif // MY_ALGO_H
#ifndef VACUUMCLEANER_H
#define VACUUMCLEANER_H

#include "House.h"
#include "common/enums.h"
#include <cmath>
#include <stack>
#include <queue>
#include <map>
#include <memory>

/**
 * The VacuumCleaner class represents a vacuum cleaner that can clean a house.
 * It keeps track of its position, battery level, and the house it is cleaning.
 */
class VacuumCleaner {
public:
    VacuumCleaner(); // Default constructor

    // Implementing the rule of five because some special member functions are needed.

    VacuumCleaner(std::shared_ptr<House> house, int maxBatterySteps); // Constructor

    VacuumCleaner(const VacuumCleaner& other); // Copy constructor

    VacuumCleaner& operator=(const VacuumCleaner& other); // Copy assignment operator

    VacuumCleaner(VacuumCleaner&& other) noexcept; // Move constructor

    VacuumCleaner& operator=(VacuumCleaner&& other) noexcept; // Move assignment operator

    ~VacuumCleaner(); // Destructor

    [[nodiscard]] std::pair<int, int> getPosition() const; // returns current position

    void move(Step step); // Move the vacuum cleaner in the specified step

    [[nodiscard]] int getSteps() const; // return steps taken

    [[nodiscard]] double getCurrentBatteryLevel() const; // Return the current battery level


    // ----------------------------------------------------- BONUS ----------------------------------------------------

    void moveBack(Step direction, double previousBatteryLevel);

private:
    std::shared_ptr<House> house; // Shared pointer to the House object
    int maxBatterySteps; // Max Battery Steps
    double currentBatteryLevel; // Current battery level
    int steps; // Number of steps taken

    std::pair<int, int> position; // Current position of the vacuum cleaner

    std::pair<int, int> dockingStation; // Coordinates of the docking station
};

#endif // VACUUMCLEANER_H
#include "VacuumCleaner.h"
#include "algo/MyAlgo.h" // This include is here to avoid circular dependency issues with the #includes.

/**
 * Default constructor
 * Initializes the vacuum cleaner with default values
 */
VacuumCleaner::VacuumCleaner()
        : house(nullptr),
          maxBatterySteps(0),
          currentBatteryLevel(0.0f),
          steps(0),
          position(0, 0),
          dockingStation(0, 0) {}


/**
 * Constructor to initialize the vacuum cleaner
 * @param house Shared pointer to the House object
 * @param maxBatterySteps Maximum steps the vacuum cleaner's battery can last
 */
VacuumCleaner::VacuumCleaner(std::shared_ptr<House> house, int maxBatterySteps)
        : house(std::move(house)), maxBatterySteps(maxBatterySteps),
          currentBatteryLevel(static_cast<float>(maxBatterySteps)),
          steps(0), position(0, 0), dockingStation(0, 0) {}


/**
 * Copy constructor
 * @param other Reference to another VacuumCleaner object to copy from
 */
VacuumCleaner::VacuumCleaner(const VacuumCleaner& other)
        : house(other.house), maxBatterySteps(other.maxBatterySteps),
          currentBatteryLevel(other.currentBatteryLevel), steps(other.steps),
          position(other.position), dockingStation(other.dockingStation) {}


/**
 * Copy assignment operator
 * @param other Reference to another VacuumCleaner object to assign from
 * @return Reference to this VacuumCleaner object
 */
VacuumCleaner& VacuumCleaner::operator=(const VacuumCleaner& other) {
    if (this != &other) {
        house = other.house;
        maxBatterySteps = other.maxBatterySteps;
        currentBatteryLevel = other.currentBatteryLevel;
        steps = other.steps;
        position = other.position;
        dockingStation = other.dockingStation;
    }
    return *this;
}


/**
 * Move constructor
 * @param other Rvalue reference to another VacuumCleaner object to move from
 */
VacuumCleaner::VacuumCleaner(VacuumCleaner&& other) noexcept
        : house(std::move(other.house)), maxBatterySteps(other.maxBatterySteps),
          currentBatteryLevel(other.currentBatteryLevel), steps(other.steps),
          position(std::move(other.position)), dockingStation(std::move(other.dockingStation)) {
    // Reset other's state
    other.maxBatterySteps = 0;
    other.currentBatteryLevel = 0;
    other.steps = 0;
}


/**
 * Move assignment operator
 * @param other Rvalue reference to another VacuumCleaner object to move from
 * @return Reference to this VacuumCleaner object
 */
VacuumCleaner& VacuumCleaner::operator=(VacuumCleaner&& other) noexcept {
    if (this != &other) {
        house = std::move(other.house);
        maxBatterySteps = other.maxBatterySteps;
        currentBatteryLevel = other.currentBatteryLevel;
        steps = other.steps;
        position = std::move(other.position);
        dockingStation = std::move(other.dockingStation);

        // Reset other's state
        other.maxBatterySteps = 0;
        other.currentBatteryLevel = 0;
        other.steps = 0;
    }
    return *this;
}


/**
 * Destructor for VacuumCleaner.
 */
VacuumCleaner::~VacuumCleaner() = default;


/**
 * @return - Returns the amount of steps the vacuum has taken.
 */
int VacuumCleaner::getSteps() const {
    return steps;
}

/**
 * @return - Returns the current position of the vacuum cleaner (relative to (0,0) in the mathematical axis which is
 * our docking station).
 * In the mathematical axis: (x, y) represents x steps right and y steps up from the origin (0, 0).
 * Assumes x and y are valid and are within the house grid when thinking of it as a mathematical axis
 * with the docking station being (0,0).
 * Note - if x is negative then (x, y) represents |x| steps left from the origin (0,0), and if y is negative then (x, y)
 * represents |y| steps down from the origin (0,0).
 * Note - We are not aware of the house structure in this class. We are only aware of our position in the mathematical
 * axis, with (0,0) being our docking station, and the conversions to the relative position in grid is done inside the
 * House class, with it not being public.
 */
std::pair<int, int> VacuumCleaner::getPosition() const {
    return position;
}

/**
 * This method moves the vacuum cleaner in an inputted step in the mathematical axis.
 * As previously explained - we are not aware of the house structure, with (0,0) representing our docking station
 * in this class, and the actual conversion to the relative position to the real docking station in grid being done
 * inside the House class, with it not being public.
 * Note - there is an assumption that this method will only be called and told to move to a valid position, with this
 * being ensured in the main cleanHouse algorithm and in the chooseNextStep and nextPossibleSteps algorithms.
 * Note - If we stay in place, we have 2 different scenarios:
 * 1. If we're staying in place and we're at the docking station and staying in place, charge the vacuum cleaner's battery.
 * In this scenario we DON'T decrease the battery level by 1 because we're charging.
 * 2. If we're staying in place and we're not at the docking station, clean the current position (We assume also here
 * that this method was not told to stay in place if the current position is clean). In this scenario we do still
 * decrease the battery level by 1 (This is also true for every other scenario except for number 1).
 * We also do the following things after moving the vacuum cleaner:
 * 1. Increase the amount of steps taken by 1.
 * 2. Update layout - We'll add the current position to the list of visited positions in the math axis (layout doesn't
 * save duplicates).
 * 3. We call the method updatePathBack(), which uses BFS to find the new shortest path back to (0,0) out of all the
 * positions we've already visited (which are saved in layout as I mentioned), and updates pathBack accordingly.
 * @param step - Step::North - Move up.
 * Step::East - Move right.
 * Step::South - Move down.
 * Step::West - Move left.
 * Step::Stay - Stay in place.
 */
void VacuumCleaner::move(Step step) {
    MyAlgo::move(step, position, currentBatteryLevel, maxBatterySteps, steps, house); // update current position, battery level and number of steps
}


/**
 * @return - Returns the current battery level of the vacuum cleaner (converted to integer, since there are no partial steps).
 */
double VacuumCleaner::getCurrentBatteryLevel() const {
    return currentBatteryLevel;
}


// ----------------------------------------------------- BONUS ----------------------------------------------------

/**
 * Move the vacuum cleaner back a step.
 * @param direction The direction in which we need to move back (the reverse of our last step)
 */
void VacuumCleaner::moveBack(Step step, double previousBatteryLevel) {
    MyAlgo::moveBack(step, previousBatteryLevel, position, currentBatteryLevel, steps, house);
}
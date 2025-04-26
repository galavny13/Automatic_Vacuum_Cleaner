#include "MyAlgo.h"

/**
 * Constructor for MyAlgorithm class.
 * Initializes member variables.
 * We don't initialize variables who have set methods. If the set methods aren't used properly, meaning the sensors
 * they receive have a problem, all the methods in the class won't work as intended.
 */
MyAlgo::MyAlgo()
        : maxSteps(0), maxBattery(0), totalDirt(0), unexploredNeighbors(0), steps(0), currentBatteryLevel(0),
        currentPosition(std::make_pair(0,0)) {}


/**
 * Sets the maximum number of steps for the simulation.
 * @param maxSteps The maximum number of steps allowed.
 */
void MyAlgo::setMaxSteps(size_t _maxSteps) {
    this->maxSteps = _maxSteps; // Set the maxSteps member variable
}


/**
 * Sets the walls sensor.
 * @param wallsSensor The WallsSensor object to be used.
 * @throws std::invalid_argument if the passed object is not a ConcreteWallsSensor.
 */
void MyAlgo::setWallsSensor(const WallsSensor& _wallsSensor) {
    this->wallsSensor = &_wallsSensor;
}


/**
 * Sets the dirt sensor.
 * @param dirtSensor The DirtSensor object to be used.
 * @throws std::invalid_argument if the passed object is not a ConcreteDirtSensor.
 */
void MyAlgo::setDirtSensor(const DirtSensor& _dirtSensor) {
    this->dirtSensor = &_dirtSensor;
}


/**
 * Sets the battery meter.
 * @param batteryMeter The BatteryMeter object to be used.
 * @throws std::invalid_argument if the passed object is not a ConcreteBatteryMeter.
 */
void MyAlgo::setBatteryMeter(const BatteryMeter& _batteryMeter) {
    this->batteryMeter = &_batteryMeter;
    maxBattery = batteryMeter->getBatteryState(); // The initial battery level is maxBattery - and it's always an integer.
    currentBatteryLevel = maxBattery;
}


/**
 * This is the nextStep method. It just ensure for the bonus that the current state's battery is updated.
 * It then returns the Step from the executeNextStep that the concrete class that inherits from this class implements.
 * @return
 */
Step MyAlgo::nextStep() {
    if (!simulationMode) {
        // In the bonus, if we're running in simulation mode - we update the houseMap after nextStep is already finished
        // and the vacuumCleaner has moved, in order to get a smoother simulation
        updateHouseMap(currentPosition);
    } else {
        currentState.previousBatteryLevel = currentBatteryLevel;
    }

    Step nextStep = executeNextStep();

    if (nextStep != Step::Finish) {
        move(nextStep, currentPosition, currentBatteryLevel, maxBattery, steps); // update current position, battery level and number of steps
    }
    return nextStep;
}


/**
 * Determines the next step for the vacuum cleaner.
 * This method is the main decision-making function that controls the vacuum's behavior.
 * It handles exploration, cleaning, and returning to the dock as needed.
 * @return The next Step the vacuum should take.
 */
Step MyAlgo::executeNextStep() {
    auto remainingSteps = maxSteps - steps; // Calculate remaining steps

    // Check if we should finish cleaning
    auto shouldFin = shouldFinish(currentPosition, remainingSteps);
    if (shouldFin.first) {
        if (currentPosition == home) {
            return Step::Finish;
        } else {
            if (shouldFin.second) { // If we can stay and clean for 1 step before returning, do so.
                houseMap[currentPosition]--;
                totalDirt--;
                return Step::Stay;
            }
            return moveTowardsDock(currentPosition); // We've got to return to dock immediately
        }
    }

    // Check if we need to return to the docking station
    auto shouldReturn = shouldReturnToDock(currentPosition, currentBatteryLevel);
    if (shouldReturn.first) {
        if (shouldReturn.second) { // If we can stay and clean for 1 step before returning, do so.
            houseMap[currentPosition]--;
            totalDirt--;
            return Step::Stay;
        }
        return moveTowardsDock(currentPosition); // We've got to return to dock immediately
    }

    bool pathPlanned = false; // Make sure we don't plan the path twice for no reason

    // If we're at the docking station
    if (currentPosition == home) {
        if (batteryMeter->getBatteryState() < maxBattery) { // If we don't have full battery - stay and charge.
            return Step::Stay;
        } // We have full battery
        if (!inExplorationMode) { // If we are in cleaning mode - replan the cleaning path, since we've had to stay and charge.
            planCleaningPath();
            pathPlanned = true;
        }
    }

    // Decide whether to switch between exploration and cleaning modes
    if (shouldSwitchMode()) {
        if (inExplorationMode) { // Switch to cleaning mode
            inExplorationMode = false;
            if (!pathPlanned) { // Don't plan path twice
                planCleaningPath(); // Replan when switching to cleaning mode
            }
        } else { // Switch to exploration mode
            inExplorationMode = true;
        }
    }

    // Execute the appropriate action based on the current mode
    if (inExplorationMode) {
        return exploreNextStep(currentPosition);
    } else {
        return followCleaningPath(currentPosition);
    }
}


/**
 * Updates the house map with current position information.
 * This method adds new positions to the house map, checks for walls,
 * and determines if a position is cleanable based on battery constraints.
 * It also efficiently updates the distances to the docking station for affected positions.
 *
 * @param pos The current position of the vacuum.
 */
void MyAlgo::updateHouseMap(const std::pair<int, int>& pos) {
    // These are only relevant for the bonus
    exploredNewCell = false;
    exploredNewNorthernWall = false;
    exploredNewEasternWall = false;
    exploredNewSouthernWall = false;
    exploredNewWesternWall = false;

    // Check if the position is not already in the house map
    if (houseMap.find(pos) == houseMap.end()) {
        exploredNewCell = true; // We've explored a new cell

        auto dirtLevel = dirtSensor->dirtLevel(); // Get the dirt level at the current position
        houseMap[pos] = dirtLevel; // Update dirt level for current position

        // Calculate the path back to the docking station for the new position
        auto pathToDock = findPath(pos, home);
        int distanceToDockingStation = pathToDock.size();
        distanceToDock[pos] = distanceToDockingStation; // Store the distance to dock for the new position

        // Check if we can reach this position, clean for one step, and return to dock
        if (2 * distanceToDockingStation + 1 <= maxBattery) {
            totalDirt += dirtLevel; // Add to the total dirt count
            cleanablePositions[pos] = true;
        } else {
            cleanablePositions[pos] = false; // Mark as not cleanable
        }

        // Check for walls in all directions and update distances
        for (const auto& direction : {Direction::North, Direction::East, Direction::South, Direction::West}) {
            if (wallsSensor->isWall(direction)) {
                auto wallPos = getNeighborPosition(pos, direction);

                if (houseMap.find(wallPos) == houseMap.end()) { // Only update if it's a new unexplored wall
                    houseMap[wallPos] = -1; // Mark wall positions with -1
                    cleanablePositions[wallPos] = false; // Walls are not cleanable

                    // Update wall exploration flags
                    switch (direction) { // This is only relevant for the bonus
                        case Direction::North: exploredNewNorthernWall = true; break;
                        case Direction::East: exploredNewEasternWall = true; break;
                        case Direction::South: exploredNewSouthernWall = true; break;
                        case Direction::West: exploredNewWesternWall = true; break;
                        default: break;
                    }
                }
            } else {
                // If not a wall, check if this new position provides a shorter path to the neighbor
                auto neighborPos = getNeighborPosition(pos, direction);
                if (houseMap.find(neighborPos) != houseMap.end() && houseMap[neighborPos] != -1) {
                    updateDistanceIfShorter(neighborPos, distanceToDockingStation + 1);
                }
            }
        }

        // Update cleanablePositions for all known positions
        for (const auto& [position, isCleanable] : cleanablePositions) {
            if (!isCleanable && position != pos) {
                auto updatedDistance = distanceToDock[position];
                bool isNowCleanable = (2 * updatedDistance + 1 <= maxBattery);
                if (isNowCleanable && houseMap[position] != -1) {
                    cleanablePositions[position] = true;
                    totalDirt += houseMap[position];
                }
            }
        }

        updateExplorationMetrics(); // Update metrics used for exploration/cleaning decisions
    }

    if (simulationMode) {
        updateStateHistory(); // Save the previous state of the algorithm in case we want to step back
    }
}


/**
 * Updates the distance to the docking station for a position if a shorter path is found.
 * This method also propagates the update to neighboring positions if necessary.
 *
 * @param pos The position to update.
 * @param newDistance The potential new shortest distance to the docking station.
 */
void MyAlgo::updateDistanceIfShorter(const std::pair<int, int>& pos, int newDistance) {
    // Check if this is a new position or if the new distance is shorter
    if (distanceToDock.find(pos) == distanceToDock.end() || newDistance < distanceToDock[pos]) {
        int oldDistance = distanceToDock[pos];
        distanceToDock[pos] = newDistance; // Update the distance

        // Update cleanability status
        bool wasCleanable = cleanablePositions[pos];
        bool isNowCleanable = (2 * newDistance + 1 <= maxBattery);
        if (wasCleanable != isNowCleanable) {
            cleanablePositions[pos] = isNowCleanable;
            if (isNowCleanable) {
                totalDirt += houseMap[pos]; // Add to total dirt if now cleanable
            } else {
                totalDirt -= houseMap[pos]; // Subtract from total dirt if no longer cleanable
            }
        }

        // Propagate the update to neighbors
        for (const auto& direction : {Direction::North, Direction::East, Direction::South, Direction::West}) {
            auto neighborPos = getNeighborPosition(pos, direction);
            if (houseMap.find(neighborPos) != houseMap.end() && houseMap[neighborPos] != -1) {
                updateDistanceIfShorter(neighborPos, newDistance + 1);
            }
        }
    }
}


/**
 * Updates metrics used for making decisions about exploration and cleaning.
 * This method calculates the number of unexplored neighboring cells.
 */
void MyAlgo::updateExplorationMetrics() {
    unexploredNeighbors = 0; // Reset unexplored neighbors count

    // Iterate through all known positions in the house map
    for (const auto& [pos, dirtLevel] : houseMap) {
        if (dirtLevel != -1) { // If not a wall
            // Check all neighbors of the current position
            for (const auto& neighbor : getNeighbors(pos)) {
                if (houseMap.find(neighbor) == houseMap.end()) {
                    unexploredNeighbors++; // Increment if neighbor is unexplored
                }
            }
        }
    }
}


/**
 * Checks if the vacuum should return to the docking station.
 * @param currentPos The current position of the vacuum.
 * @param currentBattery The current battery level.
 * @return A pair of booleans. The first indicates if the vacuum should return to dock,
 *         the second is true only if we're at the edge case where we can either stay in place for ONLY 1 step, and then
 *         return to the docking station, or we can immediately return to the docking station.
 */
std::pair<bool, bool> MyAlgo::shouldReturnToDock(const std::pair<int, int>& currentPos, double currentBattery) const {
    std::vector<Step> wayBack = findPath(currentPos, home);
    int stepsToHome = wayBack.size();

    // Check if we need to start heading back to the docking station
    if (!wayBack.empty() && stepsToHome >= currentBattery) {
        return {true, false};
    }

    // Check if we can only stay in place or start moving back to docking station
    if (!wayBack.empty() && stepsToHome >= currentBattery - 1.0) {
        if (dirtSensor->dirtLevel() == 0) { // If we can't stay and clean, return to dock
            return {true, false};
        } else { // If we can stay and clean, stay one step, and then return to dock
            return {true, true};
        }
    }

    return {false, false};
}


/**
 * Determines if the cleaning should be finished based on remaining steps, dirt, and exploration status.
 * @param currentPos The current position of the vacuum.
 * @param remainingSteps The number of steps remaining.
 * @return A pair of booleans. The first indicates if the algorithm should finish, meaning either return Step::Finish
 *         if we're at the docking station, or start going back towards the docking station in the current or next step.
 *         The second is true only if we're at the edge case where we can either stay in place for ONLY 1 step, and then
 *         return to the docking in order to finish the algorithm, or we can immediately return to the docking station.
 */
std::pair<bool, bool> MyAlgo::shouldFinish(const std::pair<int, int>& currentPos, size_t remainingSteps) const {
    // If the house is clean and fully explored, we're done
    if (isHouseClean() && isExplorationComplete()) {
        return {true, false};
    }

    auto stepsToHome = findPath(currentPos, home).size();
    auto availableSteps = remainingSteps - stepsToHome;

    // If we don't have enough steps to get back home, we should finish
    if (availableSteps <= 0) { // This shouldn't be less than 0.
        return {true, false};
    }

    // If exploration is not complete, we shouldn't finish
    if (!isExplorationComplete()) {
        return {false, false};
    }

    // Check if we can clean any more dirt
    for (const auto& [pos, dirtLevel] : houseMap) {
        if (dirtLevel > 0) {
            auto stepsToPos = findPath(currentPos, pos).size();
            auto stepsBackHome = findPath(pos, home).size();

            if (stepsToPos + dirtLevel + stepsBackHome <= availableSteps) {
                return {false, false};  // We can still clean at least one more dirty spot
            }
        }
    }

    // Check if we can only stay in place or start moving back to docking station
    if (stepsToHome >= remainingSteps - 1) {
        if (dirtSensor->dirtLevel() == 0) { // If we can't stay and clean, return to dock
            return {true, false};
        } else { // If we can stay and clean, stay one step, and then return to dock
            return {true, true};
        }
    }

    return {true, false};  // Exploration is complete and no more dirt can be cleaned within the remaining steps
}


/**
 * Moves the vacuum towards the docking station.
 * @param currentPos The current position of the vacuum.
 * @return The next Step towards the docking station.
 */
Step MyAlgo::moveTowardsDock(const std::pair<int, int>& currentPos) {
    auto path = findPath(currentPos, home);
    return path.empty() ? Step::Stay : path.front();
}


/**
 * Helper method to check if more exploration is possible from a given position. Uses BFS to check if we can explore
 * more positions.
 * @param startPos The starting position for exploration
 * @param availableBattery The available battery for exploration
 * @param availableSteps The available steps for exploration
 * @return true if more exploration is possible, false otherwise
 */
bool MyAlgo::canExploreMore(const std::pair<int, int>& currentPos, int availableBattery, int availableSteps) const {
    std::queue<std::pair<int, int>> toExplore;  // Queue for BFS
    std::set<std::pair<int, int>> visited;  // Set to track visited positions

    toExplore.push(currentPos);  // Start BFS from the given position
    visited.insert(currentPos);

    while (!toExplore.empty() && availableSteps > 0) {
        auto current = toExplore.front();
        toExplore.pop();

        for (const auto& neighbor : getNeighbors(current)) {
            if (houseMap.find(neighbor) == houseMap.end()) {  // If neighbor is unexplored
                // Calculate path length to this position
                int pathToCurrent = (current == currentPos) ? 0 : findPath(currentPos, current).size();
                // Calculate path length back to docking station
                int pathBack = (current == home) ? 0 : findPath(current, home).size();
                // Total path length including the step to the unexplored neighbor
                int totalPathLength = pathToCurrent + pathBack + 1;

                // Check if we can reach this unexplored position, clean, and return within constraints
                if (totalPathLength < availableBattery && totalPathLength < availableSteps) {
                    return true;  // Found an accessible unexplored position
                }
            }
            else if (houseMap.at(neighbor) != -1 && visited.find(neighbor) == visited.end()) {
                toExplore.push(neighbor);  // Add unexplored, non-wall neighbors to the queue
                visited.insert(neighbor);
            }
        }

        availableSteps--;  // Decrease available steps for each position checked
    }

    return false;  // No more exploration possible within constraints
}


/**
 * Checks if the house is clean.
 * @return true if the house is clean and fully explored, false otherwise.
 */
bool MyAlgo::isHouseClean() const {
    return totalDirt == 0 && isExplorationComplete();
}


/**
 * Checks if the exploration phase is complete.
 * This method determines if we've explored all accessible areas of the house
 * by checking from the current position and considering a full recharge at the dock.
 * @return true if exploration is complete, false if there are still accessible unexplored positions.
 */
bool MyAlgo::isExplorationComplete() const {
    int remainingSteps = maxSteps - steps;

    // Check if we can explore more from the current position
    if (canExploreMore(currentPosition, currentBatteryLevel, remainingSteps)) {
        return false;
    }

    // Calculate steps to return to dock
    auto pathToDock = findPath(currentPosition, home);
    // Calculate path length back to docking station
    int stepsToReturn = (currentPosition == home) ? 0 : pathToDock.size();

    // Calculate remaining battery after returning to dock
    int batteryAfterReturn = currentBatteryLevel - stepsToReturn;

    // Calculate steps needed to fully charge. Note - since we always recharge until we have full battery,
    // the inner battery state of vacuumCleaner will always be an integer, so we don't have to worry about
    // currentBatteryLevel rounding down the real battery level.
    double chargePerStep = static_cast<double>(maxBattery) / 20.0;
    int stepsToCharge = static_cast<int>(std::ceil((maxBattery - batteryAfterReturn) / chargePerStep));

    // Total steps to return and charge
    int stepsToReturnAndCharge = stepsToReturn + stepsToCharge;

    // Check if we have enough steps to return, charge, and potentially explore more
    if (stepsToReturnAndCharge < remainingSteps) {
        int stepsAfterCharge = remainingSteps - stepsToReturnAndCharge;
        if (canExploreMore(home, maxBattery, stepsAfterCharge)) {
            return false;
        }
    }

    return true; // Exploration is complete
}


/**
 * Finds the shortest path between two positions using BFS.
 * @param start The starting position.
 * @param goal The goal position.
 * @return A vector of Steps representing the path.
 */
std::vector<Step> MyAlgo::findPath(const std::pair<int, int>& start, const std::pair<int, int>& goal) const {
    std::queue<std::pair<int, int>> q; // Queue for BFS
    std::map<std::pair<int, int>, std::pair<int, int>> parent; // Map to store parent positions
    std::map<std::pair<int, int>, Step> steps; // Map to store steps taken

    q.push(start); // Start BFS from the starting position
    parent[start] = start;

    while (!q.empty()) {
        auto current = q.front(); // Get the current position
        q.pop();

        if (current == goal) break; // Stop if we've reached the goal

        // Check all neighboring positions in a consistent order: North, East, South, West
        const std::pair<Direction, Step> directions[] = {
                {Direction::North, Step::North},
                {Direction::East, Step::East},
                {Direction::South, Step::South},
                {Direction::West, Step::West}
        };

        for (const auto& [direction, step] : directions) {
            auto next = getNeighborPosition(current, direction); // Get the next position
            // If the next position is valid and unexplored, add it to the queue
            if (houseMap.find(next) != houseMap.end() && houseMap.at(next) != -1 && parent.find(next) == parent.end()) {
                q.push(next);
                parent[next] = current;
                steps[next] = step;
            }
        }
    }

    std::vector<Step> path; // Vector to store the path
    auto current = goal;
    // Reconstruct the path from goal to start
    while (current != start) {
        if (parent.find(current) == parent.end()) {
            // No path found
            return {};
        }
        path.push_back(steps[current]);
        current = parent[current];
    }
    std::reverse(path.begin(), path.end()); // Reverse the path to get start to goal

    return path;
}


/**
 * Gets the neighboring positions of a given position.
 * @param pos The position to get neighbors for.
 * @return A vector of neighboring positions.
 */
std::vector<std::pair<int, int>> MyAlgo::getNeighbors(const std::pair<int, int>& pos) {
    return {
            {pos.first, pos.second + 1},
            {pos.first, pos.second - 1},
            {pos.first - 1, pos.second},
            {pos.first + 1, pos.second}
    };
}


/**
 * Gets the neighbor position based on a direction.
 * @param pos The current position.
 * @param direction The direction to move.
 * @return The neighboring position in the given direction.
 */
std::pair<int, int> MyAlgo::getNeighborPosition(const std::pair<int, int>& pos, Direction direction) {
    switch (direction) {
        case Direction::North: return {pos.first, pos.second + 1};
        case Direction::East:  return {pos.first + 1, pos.second};
        case Direction::South: return {pos.first, pos.second - 1};
        case Direction::West:  return {pos.first - 1, pos.second};
        default: return pos;
    }
}


/**
 * Converts an integer index to its corresponding Direction enum value.
 * This helper function ensures correct mapping between loop indices and Direction enums.
 * @param index An integer from 0 to 3 representing a direction.
 * @return The corresponding Direction enum value.
 * @throws std::runtime_error if the index is out of range.
 */
Direction MyAlgo::indexToDirection(int index) {
    switch(index) {
        case 0: return Direction::North;
        case 1: return Direction::East;
        case 2: return Direction::South;
        case 3: return Direction::West;
        default: throw std::runtime_error("Invalid direction index in indexToDirection");
    }
}


/**
 * Converts a direction vector to its corresponding Step enum value.
 * This helper function is used to determine the appropriate Step based on a direction vector.
 * @param dir A pair of integers representing a direction vector.
 * @return The corresponding Step enum value.
 */
Step MyAlgo::vectorToStep(const std::pair<int, int>& dir) {
    if (dir == std::make_pair(0, 1))  return Step::North;
    if (dir == std::make_pair(1, 0))  return Step::East;
    if (dir == std::make_pair(0, -1)) return Step::South;
    if (dir == std::make_pair(-1, 0)) return Step::West;
    return Step::Stay;
}




#if __has_include("simulation/House.h")
/**
 * NOTE - This method was originally in the VacuumCleaner, but it was moved here to ensure the algorithm still works
 * properly with other projects, and so that it's not dependent on the VacuumCleaner class.
 * This static method moves the vacuum cleaner in an inputted step in the mathematical axis.
 * It operates on the provided position, battery level, and step count, potentially interacting with a House object.
 *
 * The coordinate system assumes (0,0) represents the docking station. The actual conversion to the relative position
 * to the real docking station in grid is done inside the House class and is not public.
 *
 * Assumptions and behaviors:
 * - This method will only be called with valid positions (ensured by the main cleanHouse algorithm,
 *   chooseNextStep, and nextPossibleSteps algorithms).
 * - When staying in place:
 *   1. At the docking station (0,0): The vacuum cleaner's battery is charged. Battery level is not decreased.
 *   2. Not at the docking station: The current position is cleaned (assuming it's not already clean).
 *      Battery level decreases by 1.
 * - For all moves except charging at the docking station, battery level decreases by 1.
 * - The number of steps taken is always increased by 1.
 *
 * Note: This static version does not update layout or pathBack. These operations, if needed, should be handled
 * externally after calling this method.
 *
 * @param step The direction to move: North (up), East (right), South (down), West (left), or Stay (in place).
 * @param position Reference to the current position, updated by this method.
 * @param currentBatteryLevel Reference to the current battery level, updated by this method.
 * @param maxBatterySteps Maximum battery capacity, used for charging.
 * @param steps Reference to the step count, incremented by this method.
 * @param house Shared pointer to a House object. Can be nullptr if no house interaction is needed.
 */
void MyAlgo::move(Step step, std::pair<int, int>& position, double& currentBatteryLevel,
                         int maxBatterySteps, int& steps, std::shared_ptr<House> house) {
    bool hasCharged = false;
    switch (step) {
        case Step::North: position.second++; break; // North
        case Step::East: position.first++; break; // East
        case Step::South: position.second--; break; // South
        case Step::West: position.first--; break; // West
        case Step::Stay: // Stay in place
            if (position == std::make_pair(0,0)) { // If we're at the docking station
                charge(currentBatteryLevel, maxBatterySteps);
                hasCharged = true;
            } else if (house != nullptr) { // We're not at the docking station
                house->clean(position.first, position.second); // Assume house object exists and has a clean method
            }
            break;
        default: break;
    }
    if (!hasCharged) { // Only decrease battery level if we're not charging
        currentBatteryLevel--; // Decrease battery level by 1
    }
    steps++; // Increase number of steps by 1
}


/**
 * Move the vacuum cleaner back a step.
 * @param direction The direction in which we need to move back (the reverse of our last step)
 */
void MyAlgo::moveBack(Step step, double previousBatteryLevel, std::pair<int, int>& position, double& currentBatteryLevel,
                             int& steps, std::shared_ptr<House> house) {
    switch (reverseStep(step)) {
        case Step::North: position.second++; break; // North
        case Step::East: position.first++; break; // East
        case Step::South: position.second--; break; // South
        case Step::West: position.first--; break; // West
        case Step::Stay: // Stay in place
            if (house != nullptr && position != std::make_pair(0,0)) { // If we're at the docking station
                house->unClean(position.first, position.second); // Assume house object exists and has a clean method
            }
            break;
        default: break;
    }
    steps--; // Decrease number of steps by 1
    currentBatteryLevel = previousBatteryLevel; // We need to do it this way in order to get over rounding issues.
}
#else
/**
 * NOTE - This method was originally in the VacuumCleaner, but it was moved here to ensure the algorithm still works
 * properly with other projects, and so that it's not dependent on the VacuumCleaner class.
 * This static method moves the vacuum cleaner in an inputted step in the mathematical axis.
 * It operates on the provided position, battery level, and step count, potentially interacting with a House object.
 *
 * The coordinate system assumes (0,0) represents the docking station. The actual conversion to the relative position
 * to the real docking station in grid is done inside the House class and is not public.
 *
 * Assumptions and behaviors:
 * - This method will only be called with valid positions (ensured by the main cleanHouse algorithm,
 *   chooseNextStep, and nextPossibleSteps algorithms).
 * - When staying in place:
 *   1. At the docking station (0,0): The vacuum cleaner's battery is charged. Battery level is not decreased.
 *   2. Not at the docking station: The current position is cleaned (assuming it's not already clean).
 *      Battery level decreases by 1.
 * - For all moves except charging at the docking station, battery level decreases by 1.
 * - The number of steps taken is always increased by 1.
 *
 * Note: This static version does not update layout or pathBack. These operations, if needed, should be handled
 * externally after calling this method.
 *
 * @param step The direction to move: North (up), East (right), South (down), West (left), or Stay (in place).
 * @param position Reference to the current position, updated by this method.
 * @param currentBatteryLevel Reference to the current battery level, updated by this method.
 * @param maxBatterySteps Maximum battery capacity, used for charging.
 * @param steps Reference to the step count, incremented by this method.
 */
void MyAlgo::move(Step step, std::pair<int, int>& position, double& currentBatteryLevel,
                         int maxBatterySteps, int& steps) {
    bool hasCharged = false;
    switch (step) {
        case Step::North: position.second++; break; // North
        case Step::East: position.first++; break; // East
        case Step::South: position.second--; break; // South
        case Step::West: position.first--; break; // West
        case Step::Stay: // Stay in place
            if (position == std::make_pair(0,0)) { // If we're at the docking station
                charge(currentBatteryLevel, maxBatterySteps);
                hasCharged = true;
            }
            break;
        default: break;
    }
    if (!hasCharged) { // Only decrease battery level if we're not charging
        currentBatteryLevel--; // Decrease battery level by 1
    }
    steps++; // Increase number of steps by 1
}


/**
 * Move the vacuum cleaner back a step.
 * @param direction The direction in which we need to move back (the reverse of our last step)
 */
void MyAlgo::moveBack(Step step, double previousBatteryLevel, std::pair<int, int>& position,
                      double& currentBatteryLevel, int& steps) {
    switch (reverseStep(step)) {
        case Step::North: position.second++; break; // North
        case Step::East: position.first++; break; // East
        case Step::South: position.second--; break; // South
        case Step::West: position.first--; break; // West
        case Step::Stay: // Stay in place
            break;
        default: break;
    }
    steps--; // Decrease number of steps by 1
    currentBatteryLevel = previousBatteryLevel; // We need to do it this way in order to get over rounding issues.
}
#endif


/**
 * NOTE - This method was originally in the VacuumCleaner, but it was moved here to ensure the algorithm still works
 * properly with other projects, and so that it's not dependent on the VacuumCleaner class.
 * This static method charges the vacuum cleaner's battery.
 * Charging behavior:
 * - Each call to this method is equivalent to staying 1 step in the docking station (0,0 in the mathematical axis).
 * - The charging rate is calculated as: ("max battery steps" / 20) per step.
 * - Multiple calls to this method simulate staying multiple steps at the docking station.
 *
 * Charging formula:
 * new battery level = current battery level + (("max battery steps" / 20) * number of charging steps)
 *
 * Note: If the calculated new battery level exceeds maxBatterySteps, it is capped at maxBatterySteps.
 *
 * @param currentBatteryLevel Reference to the current battery level, updated by this method.
 * @param maxBatterySteps The maximum battery capacity, used to calculate the charging rate and cap the charge level.
 */
void MyAlgo::charge(double& currentBatteryLevel, int maxBatterySteps) {
    currentBatteryLevel += static_cast<float>(maxBatterySteps) / 20.0f;
    if (currentBatteryLevel > static_cast<float>(maxBatterySteps)) {
        currentBatteryLevel = static_cast<float>(maxBatterySteps);
    }
}


/**
 * Get the reverse step of an inputted step.
 * @param step The step to return.
 * @return The reversed step.
 */
Step MyAlgo::reverseStep(Step step) {
    switch (step) {
        case Step::Stay: return Step::Stay;
        case Step::North: return Step::South;
        case Step::East: return Step::West;
        case Step::South: return Step::North;
        case Step::West: return Step::East;
        default: return Step::Stay;
    }
}


/**
 * Determines the next step during the exploration phase.
 * Note - this method and followCleaningPath are flagged as being part of a recursive call, even though we ensured we don't
 * actually end up in an infinite loop by dealing with every single edge case by robust testing. This fact leans on the
 * correctness of the implementation of the planCleaningPath and isExplorationComplete methods, which we also tested
 * with various edge cases to ensure they are actually correct. The reason we don't end up in an infinite loop is that
 * the only situation where that is possible is if there were no more cells to explore, but planCleaningPath failed to
 * generate a cleaning path, which should only happen once the house is clean and fully explored. BUT - if the house is
 * clean and fully explored, the if block containing shouldFinish in nextStep should have ensured the algorithm ended
 * BEFORE getting to this point, thus it's impossible we end up in an infinite loop.
 * @param currentPos The current position of the vacuum cleaner.
 * @return The next Step for exploration, or a cleaning step if exploration is complete.
 */
Step MyAlgo::exploreNextStep(const std::pair<int, int>& currentPos) {
    // Define directions: North (0,1), East (1,0), South (0,-1), West (-1,0)
    const std::pair<int, int> directions[] = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};

    // Check immediate neighbors first for unexplored and accessible cells
    for (int i = 0; i < 4; ++i) {
        // Get the direction vector for the current iteration
        const auto& dir = directions[i];
        // Calculate the neighbor's position
        auto neighbor = std::make_pair(currentPos.first + dir.first, currentPos.second + dir.second);

        // If neighbor is unexplored (not in houseMap) and not a wall, move there
        if (houseMap.find(neighbor) == houseMap.end() && !wallsSensor->isWall(indexToDirection(i))) {
            // Return the step corresponding to this direction
            return vectorToStep(dir);
        }
    }

    // If no immediate unexplored neighbors, use BFS to find nearest unexplored cell
    std::queue<std::pair<int, int>> frontier;
    std::map<std::pair<int, int>, std::pair<int, int>> cameFrom;

    // Start BFS from the current position
    frontier.push(currentPos);
    cameFrom[currentPos] = currentPos;

    // Continue BFS until we find an unexplored cell or exhaust all options
    while (!frontier.empty()) {
        // Get the next position to explore in BFS
        auto current = frontier.front();
        frontier.pop();

        // Check all neighbors of the current position
        for (const auto& dir : directions) {
            // Calculate the neighbor's position
            auto neighbor = std::make_pair(current.first + dir.first, current.second + dir.second);

            // If neighbor is unexplored, backtrack to get the first step
            if (houseMap.find(neighbor) == houseMap.end()) {
                // Backtrack until we reach a position adjacent to the start
                while (cameFrom[current] != currentPos) {
                    current = cameFrom[current];
                }
                // Calculate the direction vector from currentPos to the first step
                std::pair<int, int> dirVector = {current.first - currentPos.first, current.second - currentPos.second};
                // Convert the direction vector to a Step and return it
                return vectorToStep(dirVector);
            }

            // If neighbor is explored, not a wall, and not visited in this BFS
            if (houseMap.find(neighbor) != houseMap.end() && houseMap[neighbor] != -1 && cameFrom.find(neighbor) == cameFrom.end()) {
                // Add to BFS frontier and record where we came from
                frontier.push(neighbor);
                cameFrom[neighbor] = current;
            }
        }
    }

    // If no unexplored cells found, exploration is complete
    inExplorationMode = false;
    // Plan the cleaning path for the explored area
    planCleaningPath();
    // Start following the cleaning path
    return followCleaningPath(currentPos);
}


/**
 * Determines whether the vacuum should switch between exploration and cleaning modes.
 * This method balances exploration and cleaning based on local dirt density and unexplored areas,
 * taking into account the different scales of dirt levels and unexplored cell counts.
 * @return true if the vacuum should switch modes, false otherwise.
 */
bool MyAlgo::shouldSwitchMode(double x, double y) {
    // If exploration is complete, we should be in cleaning mode
    if (isExplorationComplete()) {
        return inExplorationMode;
    }

    int dirt = houseMap[currentPosition];
    int totalHouseDirt = dirt;
    int cellsWithDirt = houseMap[currentPosition] > 0 ? 1 : 0;
    int unexploredCount = 0;

    // Check nearby cells for dirt and unexplored areas
    for (const auto& neighbor : getNeighbors(currentPosition)) {
        if (houseMap.find(neighbor) == houseMap.end()) {
            unexploredCount++;
        } else {
            int dirtLevel = cleanablePositions[neighbor] ? houseMap[neighbor] : 0;
            if (dirtLevel > 0) {
                totalHouseDirt += houseMap[neighbor];
                cellsWithDirt++;
            }
        }
    }

    // Calculate average dirt level (0 to 9) for cells with dirt
    double avgDirtLevel = cellsWithDirt > 0 ? static_cast<double>(totalHouseDirt) / cellsWithDirt : 0.0;

    // // Normalize unexplored count to a 0-9 scale
    // double normalizedUnexplored = std::min(9.0, unexploredCount * 2.25); // 4 unexplored cells would give a 9

    // If in exploration mode
    if (inExplorationMode) {
        // Switch to cleaning if there's significant dirt and relatively few unexplored areas
        if (dirt >= x) {
            return true;
        }
    }
        // If in cleaning mode
    else {
        // Switch to exploration if there's little dirt and relatively many unexplored areas
        if (avgDirtLevel < y) {
            return true;
        }
    }

    // Stay in current mode if neither condition is met
    return false;
}


/**
 * Plans the cleaning path for the mapped dirty areas of the house.
 * This method creates a deterministic plan for cleaning the house, taking into account
 * the current battery level, dirt distribution, and house layout. It simulates the
 * cleaning process without modifying the real house state.
 * The method ensures determinism and robustness by:
 * 1. Using a set of dirty positions (naturally ordered)
 * 2. Simulating cleaning using a dirt changes map
 * 3. Making decisions based on the simulated state at each step
 * 4. Limiting the total number of steps to maxSteps
 */
void MyAlgo::planCleaningPath() {
    cleaningPath.clear(); // Clear any existing cleaning path

    auto dirt = houseMap[currentPosition];
    if (dirt >= 3) {
        while (dirt >= 0) {
            cleaningPath.push_back(Step::Stay);
            dirt--;
        }
        return;
    }

    auto currentPos = currentPosition; // Get the current position of the vacuum cleaner
    int remainingBattery = currentBatteryLevel; // Get the current battery level
    std::map<std::pair<int, int>, int> dirtChanges; // Map to track changes in dirt levels during simulation
    std::set<std::pair<int, int>> dirtyPositions; // Set to store positions that need cleaning

    // Collect all dirty positions from the house map
    for (const auto& [pos, dirtLevel] : houseMap) {
        if (dirtLevel > 0) {
            dirtyPositions.insert(pos); // Add position to the set if it has dirt
        }
    }

    int simulatedSteps = 0; // Counter for simulated steps taken

    while (!dirtyPositions.empty() && remainingBattery > 0 && simulatedSteps < maxSteps) {
        // Find the best next position to clean
        auto nextPos = findBestNextPosition(currentPos, dirtyPositions, remainingBattery, dirtChanges);

        if (nextPos == currentPos) { // From testing - this leads to better performance in terms of number of cleaning steps.
            break;
        }

        // Calculate path to next position
        auto pathToNext = findPath(currentPos, nextPos);

        // Get the simulated dirt level at the next position
        int dirtLevel = cleanablePositions[nextPos] ? houseMap[nextPos] : 0;
        int simulatedDirt = dirtLevel - dirtChanges[nextPos];

        // Check if we have enough battery and steps to reach and clean this position
        int batteryNeeded = pathToNext.size() + std::min(simulatedDirt, remainingBattery - static_cast<int>(pathToNext.size()));
        if (batteryNeeded > remainingBattery || simulatedSteps + batteryNeeded > maxSteps) {
            break; // Not enough battery or would exceed maxSteps, stop planning
        }

        cleaningPath.insert(cleaningPath.end(), pathToNext.begin(), pathToNext.end()); // Add path to next position

        // Add cleaning steps
        int cleaningSteps = std::min(simulatedDirt, remainingBattery - static_cast<int>(pathToNext.size()));
        cleaningSteps = std::min(cleaningSteps, static_cast<int>(maxSteps) - simulatedSteps - static_cast<int>(pathToNext.size()));
        for (int i = 0; i < cleaningSteps; ++i) {
            cleaningPath.push_back(Step::Stay);
        }

        // Update current position and remaining battery
        currentPos = nextPos;
        remainingBattery -= batteryNeeded;
        simulatedSteps += pathToNext.size() + cleaningSteps;

        dirtChanges[nextPos] += cleaningSteps; // Update simulated dirt changes
        dirtyPositions.erase(dirtyPositions.find(nextPos)); // Remove cleaned position from dirtyPositions
    }
}



/**
 * Follows the planned cleaning path, updating it as steps are taken.
 * Note - this method and exploreNextStep are flagged as being part of a recursive call, even though we ensured we don't
 * actually end up in an infinite loop by dealing with every single edge case by robust testing. This fact leans on the
 * correctness of the implementation of the planCleaningPath and isExplorationComplete methods, which we also tested
 * with various edge cases to ensure they are actually correct. The reason we don't end up in an infinite loop is that
 * the only situation where that is possible is if we started running followCleaningPath but cleaningPath was empty,
 * and we then went on to explore another step, but there were no more steps to explore (which means isExplorationComplete
 * should have returned true). Also, we won't end up in a recursive call with ourself, since we check before calling
 * ourself again that cleaningPath isn't empty, which means we won't end up in an infinite loop with our self.
 * @param currentPos The current position of the vacuum.
 * @throws std::runtime_error if we're at the edge case where planCleaningPath couldn't generate a cleaning path from
 * our current position, even though exploration is complete, and the house isn't clean, which shouldn't be possible
 * (we assume we will never reach this point, but throw a runtime error just in case).
 * We also throw a runtime error if we're at the edge case (which should also be impossible) planCleaningPath was empty,
 * and exploration is complete, and the house is clean. This shouldn't be possible since the if block containing
 * shouldFinish in nextStep is supposed to ensure the algorithm ends before reaching this point.
 * (we assume we will never reach this point, but throw a runtime error just in case).
 * @return The next Step in the cleaning path.
 */
Step MyAlgo::followCleaningPath(const std::pair<int, int>& currentPos) {
    if (cleaningPath.empty()) { // planCleaningPath returned an empty path
        if (!isExplorationComplete()) { // If we didn't finish exploring
            // Switch back to exploration mode
            inExplorationMode = true;
            return exploreNextStep(currentPos);
        }

        int dirtLevel = cleanablePositions[currentPos] ? houseMap[currentPos] : 0;

        if (dirtLevel > 0) { // If we did finish exploring, and the cleaning path was empty - stay and clean the current position if possible
            // There's dirt, clean it
            houseMap[currentPos]--;
            totalDirt--;
            return Step::Stay;
        }

        if (!isHouseClean()) { // If the cleaningPath is empty, and the house isn't clean yet
            // Generate a new cleaning path, which shouldn't be empty. If it's empty, it indicates a problem in planCleaningPath.
            planCleaningPath();
            if (cleaningPath.empty()) {
                // This should never happen, if it happened, it means there was a problem with the algorithm.
                throw std::runtime_error("planCleaningPath couldn't generate a cleaning path, despite exploration being complete.");
            }
            // We assume that our algorithm works well (due to robust testing),
            // so we won't end up in an endless loop, despite the recursive call.
            return followCleaningPath(currentPos);
        }

        // If there's no dirt at the current position, and planCleaningPath returned an empty cleaning path,
        // this indicates a problem, because exploration is complete, and the house is clean,
        // but we can't generate a valid cleaning path, so throw a runtime error.
        // This should never happen, if it happened, it means there was a problem with the algorithm.
        throw std::runtime_error("planCleaningPath couldn't generate a cleaning path, despite exploration being complete.");
    }

    Step nextStep = cleaningPath.front();
    auto nextPos = getNeighborPosition(currentPos, static_cast<Direction>(nextStep));

    if (currentPos == nextPos) {
        // We're already at the next position (this is a cleaning step)
        int dirtLevel = cleanablePositions[currentPos] ? houseMap[currentPos] : 0;
        if (dirtLevel > 0) {
            // If there's dirt, clean it
            houseMap[currentPos]--;
            totalDirt--;
            return Step::Stay;
        } else {
            // If there's no dirt, remove this step and get the next one
            cleaningPath.erase(cleaningPath.begin());
            return followCleaningPath(currentPos); // Recursive call to get the next step
        }
    } else {
        // We're moving to the next position
        cleaningPath.erase(cleaningPath.begin()); // Remove the step we're about to take
        return nextStep;
    }
}


/**
 * Finds the best next position to clean based on a heuristic.
 * This method evaluates all dirty positions and selects the one with the highest
 * score based on simulated dirt level, distance, remaining battery, and nearby dirt density.
 * @param currentPos The current position of the vacuum cleaner.
 * @param dirtyPositions A set of all remaining dirty positions.
 * @param remainingBattery The remaining battery power of the vacuum cleaner.
 * @param dirtChanges A map tracking the simulated cleaning done so far.
 * @return The position deemed best to clean next.
 */
std::pair<int, int> MyAlgo::findBestNextPosition(const std::pair<int, int>& currentPos, const std::set<std::pair<int, int>>& dirtyPositions,
                                                       int remainingBattery, const std::map<std::pair<int, int>, int>& dirtChanges) {
    std::pair<int, int> bestPos = currentPos;
    double bestScore = -1;

    for (const auto& pos : dirtyPositions) {
        auto path = findPath(currentPos, pos);
        int distance = path.size();
        int dirtLevel = cleanablePositions[pos] ? houseMap[pos] : 0;
        int simulatedDirt = dirtLevel - (dirtChanges.count(pos) ? dirtChanges.at(pos) : 0);

        // Check if we have enough battery to reach and clean this position
        if (distance >= remainingBattery) {
            continue;  // Skip this position if we can't reach it
        }

        // Calculate how much dirt we can clean at this position and its neighbors
        int batteryForCleaning = remainingBattery - distance;
        int cleanableDirt = std::min(simulatedDirt, batteryForCleaning);

        // Check nearby dirt density
        int nearbyDirt = 0;
        for (const auto& neighbor : getNeighbors(pos)) {
            if (houseMap.find(neighbor) != houseMap.end()) {
                int neighborsDirtLevel = cleanablePositions[neighbor] ? houseMap[neighbor] : 0;
                int neighborSimulatedDirt = neighborsDirtLevel - (dirtChanges.count(neighbor) ? dirtChanges.at(neighbor) : 0);
                nearbyDirt += neighborSimulatedDirt;
            }
        }

        // Calculate score based on cleanable dirt, distance, and remaining battery
        double score = static_cast<double>(cleanableDirt) / (distance + 1);
        score *= static_cast<double>(remainingBattery) / (distance + cleanableDirt);
        score += static_cast<double>(nearbyDirt) / 10; // Consider nearby dirt with less weight

        // Deterministic tie-breaking
        if (score > bestScore || (score == bestScore &&
                                  (pos.first < bestPos.first || (pos.first == bestPos.first && pos.second < bestPos.second)))) {
            bestScore = score;
            bestPos = pos;
        }
    }

    return bestPos;
}



// ----------------------------------------------------- BONUS ----------------------------------------------------

/**
 * Update our state history so we can step back if we're in simulation mode.
 */
void MyAlgo::updateStateHistory() {
    // Update all other state variables except battery
    currentState.cleanablePositions = cleanablePositions;
    currentState.distanceToDock = distanceToDock;
    currentState.inExplorationMode = inExplorationMode;
    currentState.exploredNewCell = exploredNewCell;
    currentState.exploredNewNorthernWall = exploredNewNorthernWall;
    currentState.exploredNewEasternWall = exploredNewEasternWall;
    currentState.exploredNewSouthernWall = exploredNewSouthernWall;
    currentState.exploredNewWesternWall = exploredNewWesternWall;

    // Add the current state to history
    stateHistory.push_back(currentState);
}


/**
 * Makes the algorithm run in simulation mode.
 */
void MyAlgo::enableSimulationMode() {
    simulationMode = true;
}

// Getters

std::map<std::pair<int, int>, int> &MyAlgo::getHouseMap() {
    return houseMap;
}

std::vector<Step> &MyAlgo::getCleaningPath() {
    return cleaningPath;
}

int& MyAlgo::getTotalDirt() {
    return totalDirt;
}

std::pair<int, int> &MyAlgo::getCurrentPosition() {
    return currentPosition;
}

double& MyAlgo::getCurrentBatteryLevel() {
    return currentBatteryLevel;
}

int& MyAlgo::getSteps() {
    return steps;
}

std::vector<MyAlgo::StepState> &MyAlgo::getStateHistory() {
    return stateHistory;
}

std::map<std::pair<int, int>, bool> &MyAlgo::getCleanablePositions() {
    return cleanablePositions;
}

bool& MyAlgo::getInExplorationMode() {
    return inExplorationMode;
}

std::map<std::pair<int, int>, int> &MyAlgo::getDistanceToDock(){
    return distanceToDock;
}

bool& MyAlgo::getExploredNewWesternWall() {
    return exploredNewWesternWall;
}

bool& MyAlgo::getExploredNewSouthernWall() {
    return exploredNewSouthernWall;
}

bool& MyAlgo::getExploredNewEasternWall() {
    return exploredNewEasternWall;
}

bool& MyAlgo::getExploredNewNorthernWall() {
    return exploredNewNorthernWall;
}

bool& MyAlgo::getExploredNewCell() {
    return exploredNewCell;
}
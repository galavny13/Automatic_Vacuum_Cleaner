#ifndef CONCRETE_WALLS_SENSOR_H
#define CONCRETE_WALLS_SENSOR_H

#include "common/WallSensor.h"
#include "House.h"
#include "VacuumCleaner.h"
#include <memory>
#include <vector>


/**
 * This class provides a concrete implementation of the WallsSensor abstract class.
 */
class ConcreteWallsSensor : public WallsSensor {
public:
    ConcreteWallsSensor() : house(nullptr), vacuumCleaner(nullptr) {} // Default constructor

    // Implementing the rule of five because some special member functions are needed.

    // Constructor that initializes the sensor with a house and vacuum cleaner.
    ConcreteWallsSensor(std::shared_ptr<const House> house, std::shared_ptr<const VacuumCleaner> vacuumCleaner);

    explicit ConcreteWallsSensor(const WallsSensor& other); // Copy constructor that creates a ConcreteWallsSensor from another WallsSensor.

    ConcreteWallsSensor& operator=(const ConcreteWallsSensor& other); // Copy assignment operator.

    ConcreteWallsSensor(ConcreteWallsSensor&& other) noexcept; // Move constructor.

    ConcreteWallsSensor& operator=(ConcreteWallsSensor&& other) noexcept; // Move assignment operator.

    ~ConcreteWallsSensor() override; // Virtual destructor.

    bool isWall(Direction d) const override; // Checks if there's a wall in the specified direction.


private:
    std::shared_ptr<const VacuumCleaner> vacuumCleaner; // Shared pointer to the VacuumCleaner object
    std::shared_ptr<const House> house; // Shared pointer to the House object
};

#endif // CONCRETE_WALLS_SENSOR_H
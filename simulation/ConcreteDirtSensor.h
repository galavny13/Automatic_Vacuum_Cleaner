#ifndef CONCRETE_DIRT_SENSOR_H
#define CONCRETE_DIRT_SENSOR_H

#include "common/DirtSensor.h"
#include "VacuumCleaner.h"
#include "House.h"
#include <memory>
#include <vector>

/**
 * This class provides a concrete implementation of the DirtSensor abstract class.
 */
class ConcreteDirtSensor : public DirtSensor {
public:
    ConcreteDirtSensor() : house(nullptr), vacuumCleaner(nullptr) {} // Default constructor for house, with null pointers as default.

    // Implementing the rule of five because some special member functions are needed.

    // Constructor that initializes the sensor with a house and vacuum cleaner.
    ConcreteDirtSensor(std::shared_ptr<const House> house, std::shared_ptr<const VacuumCleaner> vacuumCleaner);

    explicit ConcreteDirtSensor(const DirtSensor& other); // Copy constructor that creates a ConcreteDirtSensor from another DirtSensor.

    ConcreteDirtSensor& operator=(const ConcreteDirtSensor& other); // Copy assignment operator.

    ConcreteDirtSensor(ConcreteDirtSensor&& other) noexcept; // Move constructor.

    ConcreteDirtSensor& operator=(ConcreteDirtSensor&& other) noexcept; // Move assignment operator.

    ~ConcreteDirtSensor() override; // Virtual destructor.

    int dirtLevel() const override; // Gets the dirt level at the current position of the vacuum cleaner.

private:
    std::shared_ptr<const VacuumCleaner> vacuumCleaner; // Shared pointer to the VacuumCleaner object
    std::shared_ptr<const House> house; // Shared pointer to the House object
};

#endif // CONCRETE_DIRT_SENSOR_H
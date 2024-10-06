#include "ConcreteWallsSensor.h"
#include <stdexcept>
#include <utility>


/**
 * Constructor that initializes the sensor with a house and vacuum cleaner.
 * @param house Shared pointer to the House object
 * @param vacuumCleaner Shared pointer to the VacuumCleaner object
 */
ConcreteWallsSensor::ConcreteWallsSensor(std::shared_ptr<const House> house, std::shared_ptr<const VacuumCleaner> vacuumCleaner)
        : house(std::move(house)), vacuumCleaner(std::move(vacuumCleaner))
{}


/**
 * Copy constructor that creates a ConcreteWallsSensor from another WallsSensor.
 * @param other Reference to another WallsSensor object
 */
ConcreteWallsSensor::ConcreteWallsSensor(const WallsSensor& other)
        : vacuumCleaner(nullptr), house(nullptr) { // Initialize pointers to null
    if (const auto* concrete = dynamic_cast<const ConcreteWallsSensor*>(&other)) { // Try to cast 'other' to ConcreteWallsSensor
        if (concrete->vacuumCleaner && concrete->house) { // Check if both objects are valid
            // Create new shared_ptr instances with copies of VacuumCleaner and House
            vacuumCleaner = std::make_shared<VacuumCleaner>(*concrete->vacuumCleaner);
            house = std::make_shared<House>(*concrete->house);
        } else {
            throw std::runtime_error("Source ConcreteWallsSensor not properly initialized");
        }
    } else {
        throw std::runtime_error("Cannot copy from non-ConcreteWallsSensor");
    }
}


/**
 * Copy assignment operator.
 * @param other Reference to another ConcreteWallsSensor object
 * @return Reference to this ConcreteWallsSensor
 */
ConcreteWallsSensor& ConcreteWallsSensor::operator=(const ConcreteWallsSensor& other) {
    if (this != &other) { // Check for self-assignment
        if (other.vacuumCleaner && other.house) { // Check if both objects are valid
            // Create new shared_ptr instances with copies of VacuumCleaner and House
            vacuumCleaner = std::make_shared<VacuumCleaner>(*other.vacuumCleaner);
            house = std::make_shared<House>(*other.house);
        } else {
            throw std::runtime_error("Source ConcreteWallsSensor not properly initialized");
        }
    }
    return *this;
}


/**
 * Move constructor.
 * @param other Rvalue reference to another ConcreteWallsSensor object
 */
ConcreteWallsSensor::ConcreteWallsSensor(ConcreteWallsSensor&& other) noexcept
        : vacuumCleaner(std::move(other.vacuumCleaner)), // Move the shared_ptr
          house(std::move(other.house)) // Move the shared_ptr
{}


/**
 * Move assignment operator.
 * @param other Rvalue reference to another ConcreteWallsSensor object
 * @return Reference to this ConcreteWallsSensor
 */
ConcreteWallsSensor& ConcreteWallsSensor::operator=(ConcreteWallsSensor&& other) noexcept {
    if (this != &other) { // Check for self-assignment
        vacuumCleaner = std::move(other.vacuumCleaner); // Move the shared_ptr
        house = std::move(other.house); // Move the shared_ptr
    }
    return *this;
}


/**
 * Destructor implementation.
 */
ConcreteWallsSensor::~ConcreteWallsSensor() = default; // Use default destructor


/**
 * Checks if there's a wall in the specified direction. Assumes that vacuumCleaner's current position in the house
 * is valid.
 * @param d Direction to check
 * @return true if there's a wall, false otherwise
 */
bool ConcreteWallsSensor::isWall(Direction d) const {
    if (!vacuumCleaner || !house) { // Check if both objects are valid
        throw std::runtime_error("ConcreteWallsSensor not properly initialized");
    }
    int x = vacuumCleaner->getPosition().first; // Get x-coordinate of vacuum cleaner
    int y = vacuumCleaner->getPosition().second; // Get y-coordinate of vacuum cleaner
    return house->isWall(d, x, y); // Check if there's a wall in the specified direction
}
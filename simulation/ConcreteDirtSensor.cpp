#include "ConcreteDirtSensor.h"
#include <stdexcept>
#include <utility>


/**
 * Constructor that initializes the sensor with a house and a vacuum cleaner.
 * @param house Shared pointer to the House object.
 * @param vacuumCleaner Shared pointer to the VacuumCleaner object.
 */
ConcreteDirtSensor::ConcreteDirtSensor(std::shared_ptr<const House> house, std::shared_ptr<const VacuumCleaner> vacuumCleaner)
        : vacuumCleaner(std::move(vacuumCleaner)), // Create a shared_ptr to a copy of vacuumCleaner
          house(std::move(house)) // Create a shared_ptr to a copy of house
{}


/**
 * Copy constructor that creates a ConcreteDirtSensor from another DirtSensor.
 * @param other Reference to another DirtSensor object.
 * @throws std::runtime_error If the source ConcreteDirtSensor is not properly initialized or if the source is not a ConcreteDirtSensor.
 */
ConcreteDirtSensor::ConcreteDirtSensor(const DirtSensor& other)
        : vacuumCleaner(nullptr), house(nullptr) { // Initialize pointers to null
    if (const auto* concrete = dynamic_cast<const ConcreteDirtSensor*>(&other)) { // Try to cast 'other' to ConcreteDirtSensor
        if (concrete->vacuumCleaner && concrete->house) { // Check if both objects are valid
            // Create new shared_ptr instances with copies of VacuumCleaner and House
            vacuumCleaner = std::make_shared<VacuumCleaner>(*concrete->vacuumCleaner);
            house = std::make_shared<House>(*concrete->house);
        } else {
            throw std::runtime_error("Source ConcreteDirtSensor not properly initialized");
        }
    } else {
        throw std::runtime_error("Cannot copy from non-ConcreteDirtSensor");
    }
}


/**
 * Copy assignment operator that copies another ConcreteDirtSensor.
 * @param other Reference to another ConcreteDirtSensor object.
 * @throws std::runtime_error If the source ConcreteDirtSensor is not properly initialized.
 */
ConcreteDirtSensor& ConcreteDirtSensor::operator=(const ConcreteDirtSensor& other) {
    if (this != &other) { // Check for self-assignment
        if (other.vacuumCleaner && other.house) { // Check if both objects are valid
            // Create new shared_ptr instances with copies of VacuumCleaner and House
            vacuumCleaner = std::make_shared<VacuumCleaner>(*other.vacuumCleaner);
            house = std::make_shared<House>(*other.house);
        } else {
            throw std::runtime_error("Source ConcreteDirtSensor not properly initialized");
        }
    }
    return *this;
}


/**
 * Move constructor that moves another ConcreteDirtSensor.
 * @param other Rvalue reference to another ConcreteDirtSensor object.
 */
ConcreteDirtSensor::ConcreteDirtSensor(ConcreteDirtSensor&& other) noexcept
        : vacuumCleaner(std::move(other.vacuumCleaner)), // Move the shared_ptr
          house(std::move(other.house)) // Move the shared_ptr
{}

/**
 * Move assignment operator that moves another ConcreteDirtSensor.
 * @param other Rvalue reference to another ConcreteDirtSensor object.
 * @return Reference to this ConcreteDirtSensor.
 */
ConcreteDirtSensor& ConcreteDirtSensor::operator=(ConcreteDirtSensor&& other) noexcept {
    if (this != &other) { // Check for self-assignment
        vacuumCleaner = std::move(other.vacuumCleaner); // Move the shared_ptr
        house = std::move(other.house); // Move the shared_ptr
    }
    return *this;
}


/**
 * Destructor for ConcreteDirtSensor.
 */
ConcreteDirtSensor::~ConcreteDirtSensor() = default; // Use default destructor


/**
 * Returns the dirt level at the current position of the vacuum cleaner. This method assumes the current position
 * of the vacuum cleaner is always valid.
 * @throws std::runtime_error If the ConcreteDirtSensor is not properly initialized.
 * @return The dirt level at the current position.
 */
int ConcreteDirtSensor::dirtLevel() const {
    if (!vacuumCleaner || !house) { // Check if both objects are valid
        throw std::runtime_error("ConcreteDirtSensor not properly initialized");
    }
    int x = vacuumCleaner->getPosition().first; // Get x-coordinate of vacuum cleaner
    int y = vacuumCleaner->getPosition().second; // Get y-coordinate of vacuum cleaner
    return house->getDirtLevel(x, y); // Get dirt level at the current position
}
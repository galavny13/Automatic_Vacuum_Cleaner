#include "ConcreteBatteryMeter.h"
#include <stdexcept>
#include <utility>


/**
 * Constructor for ConcreteBatteryMeter.
 * @param vacuumCleaner A shared pointer to a constant VacuumCleaner object.
 */
ConcreteBatteryMeter::ConcreteBatteryMeter(std::shared_ptr<const VacuumCleaner> vacuumCleaner)
        : vacuumCleaner(std::move(vacuumCleaner)) {} // Create a shared_ptr to a copy of vacuumCleaner


/**
 * Copy constructor for ConcreteBatteryMeter.
 * @param other The BatteryMeter object to copy from.
 * @throws std::runtime_error If the source ConcreteBatteryMeter is not properly initialized or if the source is not a ConcreteBatteryMeter.
 */
ConcreteBatteryMeter::ConcreteBatteryMeter(const BatteryMeter& other) : vacuumCleaner(nullptr) { // Initialize pointer to null
    if (const auto* concrete = dynamic_cast<const ConcreteBatteryMeter*>(&other)) { // Try to cast 'other' to ConcreteBatteryMeter
        if (concrete->vacuumCleaner) { // Check if the object is valid
            // Create new shared_ptr instance with a copy of VacuumCleaner
            vacuumCleaner = std::make_shared<VacuumCleaner>(*concrete->vacuumCleaner);
        } else {
            throw std::runtime_error("Source ConcreteBatteryMeter not properly initialized");
        }
    } else {
        throw std::runtime_error("Cannot copy from non-ConcreteBatteryMeter");
    }
}


/**
 * Copy assignment operator for ConcreteBatteryMeter.
 * @param other The ConcreteBatteryMeter object to copy from.
 * @return A reference to the assigned ConcreteBatteryMeter object.
 * @throws std::runtime_error If the source ConcreteBatteryMeter is not properly initialized.
 */
ConcreteBatteryMeter& ConcreteBatteryMeter::operator=(const ConcreteBatteryMeter& other) {
    if (this != &other) { // Check for self-assignment
        if (other.vacuumCleaner) { // Check if the object is valid
            // Create new shared_ptr instance with a copy of VacuumCleaner
            vacuumCleaner = std::make_shared<VacuumCleaner>(*other.vacuumCleaner);
        } else {
            throw std::runtime_error("Source ConcreteBatteryMeter not properly initialized");
        }
    }
    return *this;
}


/**
 * Move constructor for ConcreteBatteryMeter.
 * @param other The ConcreteBatteryMeter object to move from.
 */
ConcreteBatteryMeter::ConcreteBatteryMeter(ConcreteBatteryMeter&& other) noexcept
        : vacuumCleaner(std::move(other.vacuumCleaner)) {} // Move the shared_ptr


/**
 * Move assignment operator for ConcreteBatteryMeter.
 * @param other The ConcreteBatteryMeter object to move from.
 * @return A reference to the assigned ConcreteBatteryMeter object.
 */
ConcreteBatteryMeter& ConcreteBatteryMeter::operator=(ConcreteBatteryMeter&& other) noexcept {
    if (this != &other) { // Check for self-assignment
        vacuumCleaner = std::move(other.vacuumCleaner); // Move the shared_ptr
    }
    return *this;
}


/**
 * Destructor for ConcreteBatteryMeter.
 */
ConcreteBatteryMeter::~ConcreteBatteryMeter() = default; // Use default destructor


/**
 * Gets the current battery state.
 * @return The current battery level of the vacuum cleaner.
 * @throws std::runtime_error If the ConcreteBatteryMeter is not properly initialized.
 */
std::size_t ConcreteBatteryMeter::getBatteryState() const {
    if (!vacuumCleaner) { // Check if the object is valid
        throw std::runtime_error("ConcreteBatteryMeter not properly initialized");
    }
    return static_cast<int>(std::floor(vacuumCleaner->getCurrentBatteryLevel())); // Get current battery level from the vacuum cleaner
}
#ifndef CONCRETE_BATTERY_METER_H
#define CONCRETE_BATTERY_METER_H

#include "common/BatteryMeter.h"
#include "VacuumCleaner.h"
#include <memory>

/**
 * This class provides a concrete implementation of the BatteryMeter abstract class.
 */
class ConcreteBatteryMeter : public BatteryMeter {
public:
    ConcreteBatteryMeter() : vacuumCleaner(nullptr) {} // Default constructor

    // Implementing the rule of five because some special member functions are needed.

    explicit ConcreteBatteryMeter(std::shared_ptr<const VacuumCleaner> vacuumCleaner); // Constructor that initializes the meter with a vacuum cleaner.

    explicit ConcreteBatteryMeter(const BatteryMeter& other); // Copy constructor that creates a ConcreteBatteryMeter from another BatteryMeter.

    ConcreteBatteryMeter& operator=(const ConcreteBatteryMeter& other); // Copy assignment operator.

    ConcreteBatteryMeter(ConcreteBatteryMeter&& other) noexcept; // Move constructor.

    ConcreteBatteryMeter& operator=(ConcreteBatteryMeter&& other) noexcept; // Move assignment operator.

    ~ConcreteBatteryMeter() override; // Virtual destructor.

    std::size_t getBatteryState() const override; // Gets the current battery state of the vacuum cleaner.

private:
    std::shared_ptr<const VacuumCleaner> vacuumCleaner; // Shared pointer to the VacuumCleaner object
};

#endif // CONCRETE_BATTERY_METER_H
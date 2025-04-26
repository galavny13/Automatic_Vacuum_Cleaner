#include "algo_323855288_209648815_B.h"
#include "algorithm/AlgorithmRegistration.h"

REGISTER_ALGORITHM(algo_323855288_209648815_B);

/**
 * Determines whether the vacuum should switch between exploration and cleaning modes.
 * This method balances exploration and cleaning based on local dirt density and unexplored areas,
 * taking into account the different scales of dirt levels and unexplored cell counts.
 * @return true if the vacuum should switch modes, false otherwise.
 */
bool algo_323855288_209648815_B::shouldSwitchMode() {
    return MyAlgo::shouldSwitchMode(3.0, 1.0);
}
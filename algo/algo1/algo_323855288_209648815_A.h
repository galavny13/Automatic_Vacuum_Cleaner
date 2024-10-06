#ifndef MY_ALGORITHM_H
#define MY_ALGORITHM_H

#include "../MyAlgo.h"

/**
 * algo_323855288_209648815_A class implements the AbstractAlgorithm interface.
 * It defines the behavior and decision-making process for the vacuum cleaner,
 * including navigation, dirt detection, battery management, and interaction
 * with the environment (walls and docking station).
 */
class algo_323855288_209648815_A : public MyAlgo {
private:

    // Private methods

    Step exploreNextStep(const std::pair<int, int>& currentPos) override; // Determine the next step during exploration

    Step followCleaningPath(const std::pair<int, int>& currentPos) override; // Follow the planned cleaning path

    bool shouldSwitchMode() override; // Determine if the vacuum should switch modes
};

#endif // MY_ALGORITHM_H

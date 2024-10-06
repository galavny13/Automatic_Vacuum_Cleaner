#ifndef MY_ALGORITHM2_H
#define MY_ALGORITHM2_H

#include "../MyAlgo.h"

/**
 * MyAlgorithm class implements the AbstractAlgorithm interface.
 * It defines the behavior and decision-making process for the vacuum cleaner,
 * including navigation, dirt detection, battery management, and interaction
 * with the environment (walls and docking station).
 */
class algo_323855288_209648815_B : public MyAlgo {
private:
    bool shouldSwitchMode() override; // Determine if the vacuum should switch modes
};

#endif // MY_ALGORITHM2_H

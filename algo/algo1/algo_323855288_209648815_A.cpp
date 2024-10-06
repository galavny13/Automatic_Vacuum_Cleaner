#include "algo_323855288_209648815_A.h"
#include "algorithm/AlgorithmRegistration.h"

REGISTER_ALGORITHM(algo_323855288_209648815_A);


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
Step algo_323855288_209648815_A::exploreNextStep(const std::pair<int, int>& currentPos) {
    // Check current position for dirt
    if (cleanablePositions[currentPos] && houseMap[currentPos] > 0) {
        return followCleaningPath(currentPos);
    }

    // Call the original exploreNextStep method from MyAlgo
    return MyAlgo::exploreNextStep(currentPos);
}


/**
 * Determines whether the vacuum should switch between exploration and cleaning modes.
 * This method balances exploration and cleaning based on local dirt density and unexplored areas,
 * taking into account the different scales of dirt levels and unexplored cell counts.
 * @return true if the vacuum should switch modes, false otherwise.
 */
bool algo_323855288_209648815_A::shouldSwitchMode() {
    return MyAlgo::shouldSwitchMode(3.0, 2.0);
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
Step algo_323855288_209648815_A::followCleaningPath(const std::pair<int, int>& currentPos) {
    if (cleanablePositions[currentPos] && houseMap[currentPos] > 0) { // Immediately clean if there's dirt.
        houseMap[currentPos]--;
        totalDirt--;
        return Step::Stay; // Clean immediately
    }

    // Call the original followCleaningPath method from MyAlgo
    return MyAlgo::followCleaningPath(currentPos);
}
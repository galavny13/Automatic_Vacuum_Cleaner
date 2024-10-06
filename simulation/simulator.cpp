#include "simulator.h"

/**
 * Runs the vacuum cleaner simulation.
 * This method handles both the GUI and NON-GUI modes of the simulation.
 */
 void MySimulator::run() {
    std::exception_ptr exceptionPtr = nullptr;  // Reset the exception pointer at the start of the run
    try {
        static std::atomic<bool> defaultFlag(false);
        run(defaultFlag);
    } catch (...) {
        // Capture any other exceptions
        exceptionPtr = std::current_exception();
    }

    // If an exception was caught, rethrow it
    if (exceptionPtr) {
    std::rethrow_exception(exceptionPtr);
    }
 }

 /**
  * Runs the vacuum cleaner simulation.
  * This method handles both the GUI and NON-GUI modes of the simulation.
  * @param timeoutFlag Flag to signal whether a timeout occurred.
  */
void MySimulator::run(std::atomic<bool>& timeoutFlag) {
    std::exception_ptr exceptionPtr = nullptr;  // Reset the exception pointer at the start of the run
    bool timeout = false;

    try {
        if (*useGUI) {
#if USE_GUI
            systemManager->runGUI();
#endif
        } else {
            Step nextStep;

            // Main simulation loop
            while (vacuumCleaner->getSteps() < maxSteps) {
                // Check battery level
                if (vacuumCleaner->getCurrentBatteryLevel() < 0 || vacuumCleaner->getCurrentBatteryLevel() > maxBatterySteps) {
                    throw std::runtime_error("Invalid battery level."); // This should never happen
                }

                // Check if a timout occurred
                if (timeoutFlag.load()) {
                    timeout = true;
                    *finished = false;
                    break;
                }

                // Get the next step from the algorithm
                nextStep = algorithm->nextStep();

                if (nextStep == Step::Finish) {
                    // Record final step and exit loop
                    steps->push_back(nextStep);
                    *finished = true;
                    break;
                }

                // Move the vacuum cleaner
                vacuumCleaner->move(nextStep);

                // Record the step
                steps->push_back(nextStep);
            }


            if (vacuumCleaner->getSteps() == maxSteps && nextStep != Step::Finish) {
                // If max steps were reached and, we didn't finish, get the final step
                nextStep = algorithm->nextStep();
                if (nextStep == Step::Finish) {
                    // Record the final step
                    steps->push_back(nextStep);
                    *finished = true;
                }
            }

            if (timeout) {
                *score = maxSteps * 2 + initialDirt * 300 + 2000;
            } else {
                *score = calculateScore(vacuumCleaner, house, *finished, maxSteps);
            }

            if (!summaryOnly) {
                // Export results to file
                std::string outputFileName = houseName + "-" + demangle(typeid(*algorithm).name()) + ".txt";
                std::ofstream outputFile(outputFileName);
                if (outputFile.is_open()) {
                    finalizeOutputFile(outputFile, vacuumCleaner, house, *finished, *score, steps, timeout);
                    outputFile.close();
                } else {
                    throw std::runtime_error("Unable to open file for writing."); // This should never happen
                }
            }
        }
    } catch (...) {
        // Capture any other exceptions
        exceptionPtr = std::current_exception();
    }

    // If an exception was caught, rethrow it
    if (exceptionPtr) {
        std::rethrow_exception(exceptionPtr);
    }
}


/**
* Sets the cleaning algorithm for the simulation.
* This method only works if the provided algorithm is of type MyAlgorithm.
* If the cast to MyAlgorithm fails, an exception is thrown.
* @param algo Reference to the AbstractAlgorithm to be used.
* @throws std::invalid_argument if algo is not of type MyAlgorithm.
*/
void MySimulator::setAlgorithm(AbstractAlgorithm& algo, bool _summaryOnly, bool _animationMode,
                  std::vector<std::vector<int>> _houseMap, std::string _currentHouseName,
                  std::string _houseName, int _maxSteps, int _maxBatterySteps,
                  int _rows, int _cols, std::pair<int, int> _dockingStation, int _initialDirt) {

    summaryOnly = _summaryOnly;
    animationMode = _animationMode;
    houseMap = _houseMap;  // Assign the contained value to houseMap
    currentHouseName = std::move(_currentHouseName);
    houseName = std::move(_houseName);
    maxSteps = _maxSteps;
    maxBatterySteps = _maxBatterySteps;
    rows = _rows;
    cols = _cols;
    dockingStation = _dockingStation;
    initialDirt = _initialDirt;


    // Initialize useGUI
    useGUI = std::make_shared<bool>();

    int status;
    char* demangledName = abi::__cxa_demangle(typeid(algo).name(), nullptr, nullptr, &status);
    std::string algoName = (status == 0) ? demangledName : typeid(algo).name();
    free(demangledName);

    if (animationMode) {
        std::cout << "Running simulation with the algorithm: " << algoName << ", on the house file: " << currentHouseName << std::endl;
    }

    // Initialize steps
    steps = std::make_shared<std::vector<Step>>(0);

    finished = std::make_shared<bool>(false);
    score = std::make_shared<int>(0);

    // Initialize house and vacuum cleaner as shared pointers
    house = std::make_shared<House>(houseMap, dockingStation);
    vacuumCleaner = std::make_shared<VacuumCleaner>(house, maxBatterySteps);

    // Initialize sensors if they haven't been initialized yet
    wallsSensor = std::make_unique<ConcreteWallsSensor>(house, vacuumCleaner);
    dirtSensor = std::make_unique<ConcreteDirtSensor>(house, vacuumCleaner);
    batteryMeter = std::make_unique<ConcreteBatteryMeter>(vacuumCleaner);

    // Create a shared_ptr from the reference, without taking ownership
    algorithm = std::shared_ptr<AbstractAlgorithm>(&algo, [](AbstractAlgorithm*) {});

    // Attempt to cast to MyAlgorithm
    myAlgo = std::dynamic_pointer_cast<MyAlgo>(algorithm);

    if (myAlgo) {

        // Successful cast
        myAlgo->setMaxSteps(maxSteps);
        myAlgo->setWallsSensor(*wallsSensor);
        myAlgo->setDirtSensor(*dirtSensor);
        myAlgo->setBatteryMeter(*batteryMeter);


        if (animationMode) {

#if USE_GUI
            if (SystemManager::isGUIAvailable()) {
                int mode;
                while (true) {
                    std::cout << "Enter 1 for animation mode, 2 for normal mode:\n";
                    std::string input;
                    std::getline(std::cin, input);

                    if (!input.empty()) {
                        mode = input[0] - '0';  // Convert first char to int

                        if (input.size() == 1 && (mode == 1 || mode == 2)) {
                            break;
                        }
                    }

                    std::cout << "Invalid answer. Please enter 1 or 2.\n" << std::endl;
                }

                *useGUI = mode == 1;

                systemManager.reset();
                systemManager.emplace(house, vacuumCleaner, myAlgo, finished, score, steps, useGUI, houseName, maxSteps, maxBatterySteps);
            } else {
                std::cout << "Animation mode is not available, running in normal mode.\n";
                *useGUI = false;
            }
#else
            std::cout << "Animation mode is not available, running in normal mode.\n";
            *useGUI = false;
#endif

        } else {
            *useGUI = false;
        }
    } else {
        // Successful cast
        algorithm->setMaxSteps(maxSteps);
        algorithm->setWallsSensor(*wallsSensor);
        algorithm->setDirtSensor(*dirtSensor);
        algorithm->setBatteryMeter(*batteryMeter);
    }
}


/**
 * Getter for score.
 * @return The score the inputted algorithm got on the inputted .house file.
 */
int MySimulator::getScore() const {
    return score? *score : 0;
}
#ifndef CPP_HW2_BONUS_SYSTEM_MANAGER_H
#define CPP_HW2_BONUS_SYSTEM_MANAGER_H

#include <typeinfo>
#include <cxxabi.h>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>
#include <regex>
#include <memory>
#include <utility>
#include <cstdio>
#include "simulation/VacuumCleaner.h"
#include "simulation/House.h"
#include "algo/MyAlgo.h"
#include "button.h"
#include "simulation/utils.h"

#if USE_GUI
#include <SFML/Graphics.hpp>
#include <filesystem>
#endif


class SystemManager {
public:
    SystemManager(const std::shared_ptr<House>& house, const std::shared_ptr<VacuumCleaner>& vacuumCleaner,
                                 const std::shared_ptr<MyAlgo>& myAlgo, const std::shared_ptr<bool>& finished, const std::shared_ptr<int>& score,
                                 const std::shared_ptr<std::vector<Step>>& steps, const std::shared_ptr<bool>& useGUI, std::string houseName,
                                 int maxSteps, int maxBatterySteps)
            : house(house), vacuumCleaner(vacuumCleaner), myAlgo(myAlgo),finished(finished), score(score),
              steps(steps), useGUI(useGUI), houseName(std::move(houseName)), maxSteps(maxSteps), maxBatterySteps(maxBatterySteps) {}

    void runGUI();

    void stepBack(Step lastStep);

    static bool isGUIAvailable(); // Checks if a GUI environment is available.

private:
    std::shared_ptr<House> house; // Shared pointer to the House object
    std::shared_ptr<VacuumCleaner> vacuumCleaner; // Shared pointer to the VacuumCleaner object
    std::shared_ptr<MyAlgo> myAlgo; // Shared pointer to the AbstractAlgorithm object
    std::shared_ptr<bool> finished; // Shared pointer to the finished boolean
    std::shared_ptr<int> score; // Shared pointer to the score integer
    std::shared_ptr<std::vector<Step>> steps; // Vector to store the steps taken by the vacuum cleaner
    std::shared_ptr<bool> useGUI; // Flag to indicate whether GUI is available and should be used
    std::string houseFileName; // Name of the house layout file
    std::string houseName; // Name / description of the house in the input file
    int maxSteps, maxBatterySteps;
    Step nextStep;

    // --------------------------------------------------- ANIMATION --------------------------------------------------

#if USE_GUI
    sf::RenderWindow window;
    sf::Font font; // Font used for text rendering
    bool isPaused = true; // Flag to indicate whether the simulation should pause
    bool singleStepForward = false; // Flag to indicate if we only want to make one step forward
    std::unique_ptr<Button> speedUpButton;
    std::unique_ptr<Button> speedDownButton;
    int speedLevel = 1; // Default speed level
    bool manualSpeedControl = false;
    std::array<int, 6> speedLevels = {200, 100, 50, 10, 5, 1};
    std::unique_ptr<Button> exportButton;
    std::unique_ptr<Button> quitButton;
    std::unique_ptr<Button> pauseButton; // Button to pause/resume the simulation
    std::unique_ptr<Button> stepButton; // Button to step through the simulation
    std::unique_ptr<Button> stepBackButton; // Button to step back in the simulation

    void initializeEndButtons();

    void showExportDialog();

    void exportToFile(const std::string& path, bool isDefaultPath);

    void showMessageBox(const std::string& message);

    void togglePause();

    void initializeSpeedButtons();

    void updateSpeedLevel(int steps);

    int getCurrentSleepTime() const;

    std::string getSpeedModeString() const;

    void updateButtonStates(); // Updates the state of all buttons based on the current simulation state.

    void initializeFont(); // Initializes the font for text rendering.

    void initializeGUI(); // Initializes the GUI components if a GUI environment is available.

    void drawControls(); // Draws all control elements on the simulation window.

    bool handleEvents(); // Handles all events for the simulation window.

    void initializeButtons(); // Initializes the GUI buttons.

    void handleMouseClick(const sf::Vector2f& mousePos); // Handles mouse click events.

    void revertToPreviousState(); // Reverts the simulation to the previous state.


    /**
     * Draws the current state of the house and vacuum cleaner using SFML.
     */
    void drawHouseState() const;

    /**
     * Sets the window and font pointers for SFML rendering.
     * @param w Pointer to the SFML RenderWindow
     * @param f Pointer to the SFML Font
     */
    void setWindowAndFont(sf::RenderWindow* w, sf::Font* f);

    mutable float currentCellSize = 30.0f;
    sf::RenderWindow* drawingWindow; // Pointer to the SFML window
    sf::Font* drawingFont; // Pointer to the font used for text rendering


// Function to suppress specific SFML warnings
#if defined(_WIN32) || defined(_WIN64)
    static void suppressWarnings() {
            freopen("NUL", "w", stdout);
            freopen("NUL", "w", stderr);
        }

        static void restoreWarnings() {
            freopen("CONOUT$", "w", stdout);
            freopen("CONOUT$", "w", stderr);
        }
#elif defined(__linux__) || defined(__APPLE__)
    static void supressWarnings() {
        std::freopen("/dev/null", "w", stdout);
        std::freopen("/dev/null", "w", stderr);
    }

    static void restoreWarnings() {
        std::freopen("/dev/tty", "w", stdout);
        std::freopen("/dev/tty", "w", stderr);
    }
#else
    static void suppressWarnings() {
            // Unknown OS, do nothing
        }

        static void restoreWarnings() {
            // Unknown OS, do nothing
        }
#endif

#endif
};


#endif //CPP_HW2_BONUS_SYSTEM_MANAGER_H

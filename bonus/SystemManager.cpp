#include "SystemManager.h"


#if USE_GUI // Only use this code if USE_GUI = true. USE_GUI = true if the SMFL and filesystem libraries are available.


/**
 * Runs the vacuum cleaner simulation.
 * This method handles the GUI mode of the simulation.
 */
void SystemManager::runGUI() {
    // Initialize GUI components
    initializeGUI();

    try {
        // Initialize font using the multi-platform method
        initializeFont();
    } catch (const std::runtime_error& e) {
        // Log font initialization failure
        std::cerr << "Font initialization failed: " << e.what() << std::endl;
        std::cerr << "The simulation will continue without text rendering." << std::endl;
    }

    myAlgo->enableSimulationMode();

    // Update the houseMap to get the initial drawing of the houseMap. Note - we don't update houseMap again
    // inside nextStep. We only decided to code it this way because we needed it
    // for the normal simulator's run method, but we still wanted to draw the initial state of the houseMap.
    myAlgo->updateHouseMap(vacuumCleaner->getPosition());

    // Set up GUI for the algorithm
    setWindowAndFont(&window, &font);

    // Draw the initial state before displaying the window
    drawHouseState();
    drawControls();
    window.display();

    // Now make the window visible
    window.setVisible(true);

    sf::Clock stepClock;
    bool windowClosed = false;

    // Main simulation loop
    while (window.isOpen() && vacuumCleaner->getSteps() < maxSteps) {

        // Check battery level
        if (vacuumCleaner->getCurrentBatteryLevel() < 0 || vacuumCleaner->getCurrentBatteryLevel() > maxBatterySteps) {
            std::cout << "Invalid battery level.\n" << std::endl;
            if (*useGUI) {
                window.close();
            }
            break;
        }

        // Handle user input and window events
        if (handleEvents()) {
            windowClosed = true;
            break;
        }

        // Only proceed with simulation if not paused or taking a single step
        if (!isPaused || singleStepForward) {
            // Get the next step from the algorithm
            nextStep = myAlgo->nextStep();

            if (nextStep == Step::Finish) {
                // Record final step and exit loop
                steps->push_back(nextStep);
                *finished = true;
                break;
            }

            // Move the vacuum cleaner
            vacuumCleaner->move(nextStep);

            // Update the houseMap to get the updated drawing of the houseMap. Note - we don't update houseMap again
            // inside nextStep. We only decided to code it this way because we wanted the simulation to run smoothly
            // and still work with the normal run() method in simulator.cpp.
            myAlgo->updateHouseMap(vacuumCleaner->getPosition());

            // Record the step
            steps->push_back(nextStep);

            updateSpeedLevel(vacuumCleaner->getSteps());

            // If we've taken a single step, pause again
            if (singleStepForward) {
                isPaused = true;
                singleStepForward = false;
            }
        }

        // Update the GUI
        drawHouseState();
        drawControls();
        window.display();

        // Control animation speed
        if (!isPaused) {
            updateSpeedLevel(vacuumCleaner->getSteps());
            float elapsedTime = stepClock.getElapsedTime().asMilliseconds();
            sf::sleep(sf::milliseconds(std::max(0.f, static_cast<float>(getCurrentSleepTime()) - elapsedTime)));
            stepClock.restart();
        }

        // Update button states after each step
        updateButtonStates();
    }

    if (windowClosed) {
        throw std::runtime_error("The window was closed by pressing the X button before the algorithm finished running.\n"
                                 "The simulation will prematurely end without generating the output file.\n");
    }


    if (vacuumCleaner->getSteps() == maxSteps && nextStep != Step::Finish) {
        // If max steps were reached and, we didn't finish, get the final step
        nextStep = myAlgo->nextStep();
        if (nextStep == Step::Finish) {
            // Record the final step
            steps->push_back(nextStep);
            *finished = true;
        }
    }

    *score = calculateScore(vacuumCleaner, house, *finished, maxSteps);

    // Update buttons states one last time
    updateButtonStates();

    // Initialize end buttons (export and quit)
    initializeEndButtons();

    // Keep window open until user closes it
    while (window.isOpen()) {
        handleEvents();
        drawHouseState();
        drawControls();
        exportButton->draw(window);
        quitButton->draw(window);
        window.display();
    }
}



/**
 * Reverts the algorithm state by one step.
 * @param lastStep The step to reverse.
 */
void SystemManager::stepBack(Step lastStep) {
    std::pair<int, int> home = std::make_pair(0,0);
    std::map<std::pair<int, int>, int>& houseMap = myAlgo->getHouseMap();
    std::pair<int, int>& currentPosition = myAlgo->getCurrentPosition();

    std::vector<MyAlgo::StepState>& stateHistory = myAlgo->getStateHistory();
    MyAlgo::StepState previousState = stateHistory.back();
    stateHistory.pop_back();

    // Update the actual vacuum cleaner's state
    vacuumCleaner->moveBack(lastStep, previousState.previousBatteryLevel);

    if (previousState.exploredNewCell) {
        houseMap.erase(currentPosition);
        if (previousState.exploredNewNorthernWall) {
            houseMap.erase(MyAlgo::getNeighborPosition(currentPosition, Direction::North));
        }
        if (previousState.exploredNewEasternWall) {
            houseMap.erase(MyAlgo::getNeighborPosition(currentPosition, Direction::East));
        }
        if (previousState.exploredNewSouthernWall) {
            houseMap.erase(MyAlgo::getNeighborPosition(currentPosition, Direction::South));
        }
        if (previousState.exploredNewWesternWall) {
            houseMap.erase(MyAlgo::getNeighborPosition(currentPosition, Direction::West));
        }
    } else {
        if (lastStep == Step::Stay && currentPosition != home) {
            houseMap[currentPosition]++;
            myAlgo->getTotalDirt()++;
        }
    }

    if (!previousState.inExplorationMode) {
        myAlgo->getCleaningPath().push_back(lastStep);
    }

    // Update our inner representation of the vacuum cleaner state
    MyAlgo::moveBack(lastStep, previousState.previousBatteryLevel, currentPosition, myAlgo->getCurrentBatteryLevel(), myAlgo->getSteps());

    myAlgo->getCleanablePositions() = previousState.cleanablePositions;
    myAlgo->getDistanceToDock() = previousState.distanceToDock;
    myAlgo->getInExplorationMode() = previousState.inExplorationMode;
    myAlgo->getExploredNewCell() = previousState.exploredNewCell;
    myAlgo->getExploredNewNorthernWall() = previousState.exploredNewNorthernWall;
    myAlgo->getExploredNewEasternWall() = previousState.exploredNewEasternWall;
    myAlgo->getExploredNewSouthernWall() = previousState.exploredNewSouthernWall;
    myAlgo->getExploredNewWesternWall() = previousState.exploredNewWesternWall;

    myAlgo->updateExplorationMetrics();
}


/**
 * Set the window and font for the drawHouseState method
 * @param w The window
 * @param f The font
 */
void SystemManager::setWindowAndFont(sf::RenderWindow* w, sf::Font* f) {
    drawingWindow = w;
    drawingFont = f;
}


/**
 * Draws the current state of the house using SFML.
 */
void SystemManager::drawHouseState() const {
    if (!drawingWindow) return;

    const std::map<std::pair<int, int>, int> houseMap = myAlgo->getHouseMap();
    const std::pair<int, int> currentPosition = myAlgo->getCurrentPosition();

    drawingWindow->clear(sf::Color::White);

    int minX = currentPosition.first, maxX = currentPosition.first;
    int minY = currentPosition.second, maxY = currentPosition.second;
    for (const auto& [pos, _] : houseMap) {
        minX = std::min(minX, pos.first);
        maxX = std::max(maxX, pos.first);
        minY = std::min(minY, pos.second);
        maxY = std::max(maxY, pos.second);
    }

    static float visibleMinX = minX, visibleMaxX = maxX, visibleMinY = minY, visibleMaxY = maxY;
    float transitionSpeed = 0.3f;

    visibleMinX = std::min(visibleMinX, static_cast<float>(currentPosition.first));
    visibleMaxX = std::max(visibleMaxX, static_cast<float>(currentPosition.first));
    visibleMinY = std::min(visibleMinY, static_cast<float>(currentPosition.second));
    visibleMaxY = std::max(visibleMaxY, static_cast<float>(currentPosition.second));

    visibleMinX += (minX - visibleMinX) * transitionSpeed;
    visibleMaxX += (maxX - visibleMaxX) * transitionSpeed;
    visibleMinY += (minY - visibleMinY) * transitionSpeed;
    visibleMaxY += (maxY - visibleMaxY) * transitionSpeed;

    float windowWidth = static_cast<float>(drawingWindow->getSize().x);
    float windowHeight = static_cast<float>(drawingWindow->getSize().y);
    float visibleWidth = visibleMaxX - visibleMinX + 1;
    float visibleHeight = visibleMaxY - visibleMinY + 1;
    float widthRatio = windowWidth / visibleWidth;
    float heightRatio = windowHeight / visibleHeight;
    float idealCellSize = std::min(widthRatio, heightRatio) * 0.85f;

    static float lastCellSize = idealCellSize;
    float cellSizeTransitionSpeed = std::min(1.0f, 5.0f / std::max(visibleWidth, visibleHeight));
    currentCellSize = lastCellSize + (idealCellSize - lastCellSize) * cellSizeTransitionSpeed;
    lastCellSize = currentCellSize;

    float offsetX = (windowWidth - visibleWidth * currentCellSize) / 2.0f;
    float offsetY = (windowHeight - visibleHeight * currentCellSize) / 2.0f;

    for (int y = maxY; y >= minY; --y) {
        for (int x = minX; x <= maxX; ++x) {
            std::pair<int, int> pos = {x, y};
            auto it = houseMap.find(pos);

            if (it == houseMap.end()) continue;

            sf::RectangleShape cell(sf::Vector2f(currentCellSize, currentCellSize));
            cell.setPosition(offsetX + (x - visibleMinX) * currentCellSize, offsetY + (visibleMaxY - y) * currentCellSize);
            cell.setOutlineThickness(1);
            cell.setOutlineColor(sf::Color::Black);

            if (pos == std::make_pair(0, 0)) {
                cell.setFillColor(sf::Color::Green);
            } else if (it->second == -1) {
                cell.setFillColor(sf::Color::Black);
            } else {
                int dirtLevel = it->second;
                cell.setFillColor(sf::Color(255, 255 - dirtLevel * 25, 255 - dirtLevel * 25));
            }

            drawingWindow->draw(cell);

            if (it->second >= 0 && drawingFont) {
                sf::Text dirtText;
                dirtText.setFont(*drawingFont);
                dirtText.setCharacterSize(static_cast<unsigned int>(currentCellSize * 0.7f));
                dirtText.setFillColor(sf::Color::Black);
                dirtText.setString(std::to_string(it->second));

                sf::FloatRect textRect = dirtText.getLocalBounds();
                dirtText.setOrigin(textRect.left + textRect.width / 2.0f, textRect.top + textRect.height / 2.0f);
                dirtText.setPosition(offsetX + (x - visibleMinX) * currentCellSize + currentCellSize / 2.0f,
                                     offsetY + (visibleMaxY - y) * currentCellSize + currentCellSize / 2.0f);

                drawingWindow->draw(dirtText);
            }
        }
    }

    sf::RectangleShape vacuumOverlay(sf::Vector2f(currentCellSize, currentCellSize));
    vacuumOverlay.setPosition(offsetX + (currentPosition.first - visibleMinX) * currentCellSize,
                              offsetY + (visibleMaxY - currentPosition.second) * currentCellSize);
    vacuumOverlay.setFillColor(sf::Color(128, 0, 128, 128));
    drawingWindow->draw(vacuumOverlay);

    if (drawingFont) {
        sf::Text statusText;
        statusText.setFont(*drawingFont);
        statusText.setCharacterSize(14);
        statusText.setFillColor(sf::Color::Black);
        statusText.setPosition(10, windowHeight - 70);

        std::stringstream ss;
        ss << "Position: (" << currentPosition.first << ", " << currentPosition.second << ")\n"
           << "Battery: " << myAlgo->getCurrentBatteryLevel() << "\n"
           << "Steps: " << myAlgo->getSteps();

        statusText.setString(ss.str());

        drawingWindow->draw(statusText);
    }
}


/**
 * Initializes the font for text rendering.
 */
void SystemManager::initializeFont() {
    // Try to load common fonts on different systems
    const std::vector<std::string> fontPaths = {
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",  // Ubuntu
            "/System/Library/Fonts/Helvetica.ttc",              // macOS
            "C:\\Windows\\Fonts\\arial.ttf"                     // Windows
    };

    for (const auto& path : fontPaths) {
        if (font.loadFromFile(path)) {
            return;  // Successfully loaded a font
        }
    }

    // If we couldn't load any font, throw an exception
    throw std::runtime_error("Failed to load any font");
}



/**
 * Checks if a GUI environment is available.
 * This method checks for the necessary components without creating a window.
 *
 * @return true if GUI is available, false otherwise
 */
bool SystemManager::isGUIAvailable() {
    // suppressWarnings();  // Suppress warnings

    bool available = false;

    // Check if SFML graphics module is available
    if (sf::Texture::getMaximumSize() > 0) {
        // Check for X11 display
        const char* display = std::getenv("DISPLAY");
        if (display && !std::string(display).empty()) {
            available = true;
        } else {
            std::cerr << "No X11 display available. GUI features will be disabled." << std::endl;
            std::cerr.flush();
        }
    } else {
        std::cerr << "SFML graphics module is not available. GUI features will be disabled." << std::endl;
        std::cerr.flush();
    }

    // restoreWarnings();  // Restore warnings

    if (!available) {
        std::cerr << "GUI features will be disabled." << std::endl;
        std::cerr.flush();
    }

    return available;
}

/**
 * Checks if a GUI environment is available.
 * @return true if GUI is available, false otherwise
 */
// bool SystemManager::isGUIAvailable() {
//     suppressWarnings();  // Suppress warnings

//     sf::ContextSettings settings;
//     settings.depthBits = 24;    // Minimal depth bits
//     settings.stencilBits = 8;   // Minimal stencil bits
//     settings.antialiasingLevel = 0;  // No antialiasing
//     settings.majorVersion = 3;  // Minimal OpenGL version
//     settings.minorVersion = 0;

//     // Attempt to create a small test window
//     sf::RenderWindow testWindow(sf::VideoMode(1, 1), "Test", sf::Style::None, settings);

//     bool available = testWindow.isOpen();  // Check if the window was successfully created
//     testWindow.close();  // Close the test window

//     restoreWarnings();  // Restore warnings

//     return available;
// }


/**
 * Initializes the GUI components if a GUI environment is available.
 */
void SystemManager::initializeGUI() {
    if (*useGUI) {
        *useGUI = isGUIAvailable();  // Check if GUI is available
        if (*useGUI) {
            // If GUI is available, create the SFML window
            window.create(sf::VideoMode(1152, 864), "Vacuum Cleaner Simulation", sf::Style::Titlebar | sf::Style::Close);

            // Attempt to load a font for text rendering
            if (!font.loadFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")) {
                std::cerr << "Failed to load font. Text rendering may not work correctly." << std::endl;
            }
        }
    }

    initializeButtons();
    initializeSpeedButtons();
    updateButtonStates();
}



/**
 * Initializes the GUI buttons.
 */
void SystemManager::initializeButtons() {
    unsigned int buttonTextSize = 16;  // Reduced from 18
    pauseButton = std::make_unique<Button>(sf::Vector2f(10, 10), sf::Vector2f(100, 30), "Start", font, buttonTextSize);
    stepButton = std::make_unique<Button>(sf::Vector2f(120, 10), sf::Vector2f(100, 30), "Step", font, buttonTextSize);
    stepBackButton = std::make_unique<Button>(sf::Vector2f(230, 10), sf::Vector2f(100, 30), "Step Back", font, buttonTextSize);

    pauseButton->setOnClick([this]() {
        isPaused = !isPaused;
        updateButtonStates();
    });

    stepButton->setOnClick([this]() {
        if (isPaused) {
            singleStepForward = true;
        }
    });

    stepBackButton->setOnClick([this]() {
        if (isPaused) {
            revertToPreviousState();
        }
    });
}

/**
 * Handles mouse click events.
 * @param mousePos The position of the mouse click.
 */
void SystemManager::handleMouseClick(const sf::Vector2f& mousePos) {
    if (pauseButton->isMouseOver(mousePos) && pauseButton->isEnabled()) {
        togglePause();
    } else if (stepButton->isMouseOver(mousePos) && stepButton->isEnabled()) {
        singleStepForward = true;
    } else if (stepBackButton->isMouseOver(mousePos) && stepBackButton->isEnabled()) {
        revertToPreviousState();
    } else if (speedUpButton->isMouseOver(mousePos) && speedUpButton->isEnabled()) {
        speedUpButton->handleClick();
    } else if (speedDownButton->isMouseOver(mousePos) && speedDownButton->isEnabled()) {
        speedDownButton->handleClick();
    } else if (*finished && exportButton && exportButton->isMouseOver(mousePos)) {
        exportButton->handleClick();
    } else if (*finished && quitButton && quitButton->isMouseOver(mousePos)) {
        quitButton->handleClick();
    }
}

/**
 * Handles all events for the simulation window.
 * This includes keyboard input, mouse clicks, and window closing events.
 * @return true if the window has been closed, false otherwise.
 */
bool SystemManager::handleEvents() {
    sf::Event event;
    while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
            // Close the window if the close button is clicked
            window.close();
            return true; // Indicate that the window has been closed
        }
        else if (event.type == sf::Event::KeyPressed && !*finished) {
            // Handle keyboard input only if the simulation hasn't finished
            if (event.key.code == sf::Keyboard::Space) {
                // Toggle pause state when space is pressed
                togglePause();
            } else if (event.key.code == sf::Keyboard::Right && isPaused) {
                // Step forward if paused and right arrow is pressed
                singleStepForward = true;
            } else if (event.key.code == sf::Keyboard::Left && isPaused && vacuumCleaner->getSteps() > 0) {
                // Step backward if paused, left arrow is pressed, and not at the start
                revertToPreviousState();
            }
        }
        else if (event.type == sf::Event::MouseButtonPressed) {
            // Handle mouse clicks
            handleMouseClick(window.mapPixelToCoords(sf::Mouse::getPosition(window)));
        }
    }
    return false; // Indicate that the window has not been closed
}


/**
 * Draws all control elements on the simulation window.
 * This includes buttons and status text.
 */
void SystemManager::drawControls() {
    // Draw all buttons
    pauseButton->draw(window);
    stepButton->draw(window);
    stepBackButton->draw(window);
    speedUpButton->draw(window);
    speedDownButton->draw(window);

    // Create and draw status text
    std::string statusText = "Status: ";
    if (*finished) {
        statusText += "Finished";
    } else if (vacuumCleaner->getSteps() == 0 && isPaused) {
        statusText += "Waiting to start";
    } else {
        statusText += isPaused ? "Paused" : "Running";
    }

    sf::Text status(statusText, font, 14);
    status.setPosition(10, 50);
    status.setFillColor(sf::Color::Black);
    window.draw(status);

    // Draw speed information
    sf::Text speedText;
    speedText.setFont(font);
    speedText.setCharacterSize(14);
    speedText.setFillColor(sf::Color::Black);
    speedText.setPosition(window.getSize().x - 220, 50);
    speedText.setString(getSpeedModeString());
    window.draw(speedText);
}


/**
 * Reverts the simulation to the previous state (we use this for the step back button).
 */
void SystemManager::revertToPreviousState() {
    // Check if there's a state to revert to
    if (vacuumCleaner->getSteps() == 0) {
        // If we're at the initial state, there's nothing to revert
        return;
    }

    // Revert the algorithm state
    if (myAlgo) {

        // Remove the last step from the steps vector
        if (!steps->empty()) {
            stepBack(steps->back());
            steps->pop_back();
        }

        sf::Clock stepClock;

        if (*useGUI) {
            // Update the GUI
            drawHouseState();
            drawControls();
            window.display();

            updateSpeedLevel(vacuumCleaner->getSteps());
            float elapsedTime = stepClock.getElapsedTime().asMilliseconds();
            sf::sleep(sf::milliseconds(std::max(0.f, static_cast<float>(getCurrentSleepTime()) - elapsedTime)));
            stepClock.restart();
        }
    }
}


/**
 * Initialize the buttons to control the speed level (switches us to manual mode if pressed).
 */
void SystemManager::initializeSpeedButtons() {
    unsigned int buttonTextSize = 16;
    speedDownButton = std::make_unique<Button>(sf::Vector2f(window.getSize().x - 220, 10), sf::Vector2f(100, 30), "Speed Down", font, buttonTextSize);
    speedUpButton = std::make_unique<Button>(sf::Vector2f(window.getSize().x - 110, 10), sf::Vector2f(100, 30), "Speed Up", font, buttonTextSize);

    speedUpButton->setOnClick([this]() {
        if (speedLevel < 6) {
            speedLevel++;
            manualSpeedControl = true;
        }
        updateButtonStates();
    });

    speedDownButton->setOnClick([this]() {
        if (speedLevel > 1) {
            speedLevel--;
            manualSpeedControl = true;
        }
        updateButtonStates();
    });
}


/**
 * If in automatic mode - update the speed level based on the number of steps.
 * @param steps The number of steps the vacuum cleaner has taken.
 */
void SystemManager::updateSpeedLevel(int steps) {
    if (!manualSpeedControl) {
        if (steps < 10) speedLevel = 1;
        else if (steps < 100) speedLevel = 2;
        else if (steps < 500) speedLevel = 3;
        else speedLevel = 4;
    }
}


/**
 * Get the current sleep time between each nextStep iteration based on the speed level.
 * @return The current sleep time between each nextStep iteration based on the speed level.
 */
int SystemManager::getCurrentSleepTime() const {
    return speedLevels[speedLevel - 1];
}


/**
 * Get the current speed level and mode (manual / automatic).
 * @return The current speed level and mode.
 */
std::string SystemManager::getSpeedModeString() const {
    std::stringstream ss;
    ss << "Speed: Level " << speedLevel << " (" << (manualSpeedControl ? "Manual" : "Auto") << ")";
    return ss.str();
}


/**
 * Updates the state of all buttons based on the current simulation state.
 * This includes enabling/disabling buttons and updating their text.
 */
void SystemManager::updateButtonStates() {
    if (pauseButton && stepButton && stepBackButton && speedUpButton && speedDownButton) {
        if (*finished) {
            // Disable all buttons if the simulation has finished
            pauseButton->setEnabled(false);
            stepButton->setEnabled(false);
            stepBackButton->setEnabled(false);
            speedUpButton->setEnabled(false);
            speedDownButton->setEnabled(false);
        } else {
            // Update button states based on current simulation state
            pauseButton->setEnabled(true);
            pauseButton->setText(vacuumCleaner->getSteps() == 0 ? "Start" : (isPaused ? "Resume" : "Pause"));
            stepButton->setEnabled(isPaused);
            stepBackButton->setEnabled(isPaused && vacuumCleaner->getSteps() > 0);
            speedUpButton->setEnabled(!isPaused && speedLevel < 6);
            speedDownButton->setEnabled(!isPaused && speedLevel > 1);
        }
    }
}


/**
 * Toggles the pause state of the simulation.
 * This method is called when the pause button is clicked or the space key is pressed.
 */
void SystemManager::togglePause() {
    if (!*finished) {
        isPaused = !isPaused;
        updateButtonStates();
    }
}


/**
 * Initialization for the buttons that will be displayed once the algorithm has ended.
 */
void SystemManager::initializeEndButtons() {
    unsigned int buttonTextSize = 16;
    exportButton = std::make_unique<Button>(sf::Vector2f(window.getSize().x - 220, window.getSize().y - 40), sf::Vector2f(100, 30), "Export", font, buttonTextSize);
    quitButton = std::make_unique<Button>(sf::Vector2f(window.getSize().x - 110, window.getSize().y - 40), sf::Vector2f(100, 30), "Quit", font, buttonTextSize);

    exportButton->setOnClick([this]() { showExportDialog(); });
    quitButton->setOnClick([this]() { window.close(); });
}


/**
 * Displays a dialog for exporting simulation results.
 * Allows the user to choose between default and custom file paths.
 */
void SystemManager::showExportDialog() {
    sf::RenderWindow dialog(sf::VideoMode(800, 400), "Export Options");
    sf::View view = dialog.getDefaultView();

    Button useDefaultPath(sf::Vector2f(50, 70), sf::Vector2f(250, 50), "Use Default Path", font, 20);
    Button useCustomPath(sf::Vector2f(50, 260), sf::Vector2f(250, 50), "Use Custom Path", font, 20);

    sf::Text customPathPrompt("Enter custom path:", font, 20);
    customPathPrompt.setPosition(50, 150);
    customPathPrompt.setFillColor(sf::Color::Black);

    sf::RectangleShape inputBox(sf::Vector2f(700, 40));
    inputBox.setPosition(50, 190);
    inputBox.setFillColor(sf::Color::White);
    inputBox.setOutlineThickness(2);
    inputBox.setOutlineColor(sf::Color::Black);

    sf::Text customPathText("", font, 24);
    customPathText.setFillColor(sf::Color::Black);
    customPathText.setPosition(inputBox.getPosition().x + 5, inputBox.getPosition().y + 5);

    std::string customPath;
    size_t cursorPosition = 0;
    size_t selectionStart = 0, selectionEnd = 0;
    bool isInputActive = false;
    sf::Clock clock;

    std::vector<std::string> undoHistory;
    size_t undoIndex = 0;

    auto updateInputBox = [&]() {
        customPathText.setString(customPath);
        float textWidth = customPathText.getLocalBounds().width;
        float minWidth = 700;
        float maxWidth = 1500;
        float newWidth = std::max(minWidth, std::min(maxWidth, textWidth + 20));

        inputBox.setSize(sf::Vector2f(newWidth, 40));

        unsigned int newDialogWidth = std::max(800u, std::min(1600u, static_cast<unsigned int>(newWidth + inputBox.getPosition().x + 50)));

        if (newDialogWidth != dialog.getSize().x) {
            dialog.setSize(sf::Vector2u(newDialogWidth, dialog.getSize().y));
            view.setSize(dialog.getSize().x, dialog.getSize().y);
            view.setCenter(dialog.getSize().x / 2, dialog.getSize().y / 2);
            dialog.setView(view);
        }

        customPathText.setPosition(inputBox.getPosition().x + 5,
                                   inputBox.getPosition().y + (inputBox.getSize().y - customPathText.getLocalBounds().height) / 2.0f - customPathText.getLocalBounds().top);
        customPathText.setStyle(sf::Text::Regular);
    };

    auto getCursorPositionFromMouseX = [&](float mouseX) -> size_t {
        float textStart = customPathText.getPosition().x;
        float totalWidth = 0;
        for (size_t i = 0; i < customPath.length(); ++i) {
            float charWidth = customPathText.getFont()->getGlyph(customPath[i], customPathText.getCharacterSize(), false).advance;
            if (mouseX < textStart + totalWidth + charWidth / 2) {
                return i;
            }
            totalWidth += charWidth;
        }
        return customPath.length();
    };

    auto moveWordWise = [&](bool forward, bool extending = false) {
        if (forward) {
            // If we're in the middle of a word, move to its end
            while (cursorPosition < customPath.length() && !std::isspace(customPath[cursorPosition])) {
                cursorPosition++;
            }
            // Move through whitespace to the start of the next word
            while (cursorPosition < customPath.length() && std::isspace(customPath[cursorPosition])) {
                cursorPosition++;
            }
        } else {
            if (cursorPosition > 0) {
                cursorPosition--;
                // Move through whitespace to the end of the previous word
                while (cursorPosition > 0 && std::isspace(customPath[cursorPosition])) {
                    cursorPosition--;
                }
                // Move to the start of the current word
                while (cursorPosition > 0 && !std::isspace(customPath[cursorPosition - 1])) {
                    cursorPosition--;
                }
            }
        }
        if (extending) {
            selectionEnd = cursorPosition;
        } else {
            selectionStart = selectionEnd = cursorPosition;
        }
    };

    while (dialog.isOpen()) {
        sf::Event event;
        while (dialog.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                dialog.close();
            }
            else if (event.type == sf::Event::MouseButtonPressed) {
                sf::Vector2f mousePos = dialog.mapPixelToCoords(sf::Mouse::getPosition(dialog));
                if (inputBox.getGlobalBounds().contains(mousePos)) {
                    isInputActive = true;
                    cursorPosition = getCursorPositionFromMouseX(mousePos.x);
                    selectionStart = selectionEnd = cursorPosition;
                } else {
                    isInputActive = false;
                }

                if (useDefaultPath.isMouseOver(mousePos)) {
                    std::string outputFileName = std::filesystem::absolute(houseName + "-" + demangle(typeid(*myAlgo).name()) + ".txt").string();
                    exportToFile(outputFileName, true);
                    dialog.close();
                }
                else if (useCustomPath.isMouseOver(mousePos)) {
                    if (!customPath.empty()) {
                        std::filesystem::path fsPath(customPath);
                        if (fsPath.is_absolute()) {
                            if (std::filesystem::is_directory(fsPath)) {
                                exportToFile(customPath, false);
                                dialog.close();
                            } else {
                                showMessageBox("Error: Path is not a directory");
                            }
                        } else {
                            showMessageBox("Error: Path is not absolute");
                        }
                    } else {
                        showMessageBox("Error: Invalid path");
                    }
                }
            }
            else if (event.type == sf::Event::TextEntered && isInputActive) {
                if (event.text.unicode != '\n' && event.text.unicode != '\r') {
                    if (event.text.unicode == '\t') {
                        if (selectionStart != selectionEnd) {
                            size_t start = std::min(selectionStart, selectionEnd);
                            size_t end = std::max(selectionStart, selectionEnd);
                            customPath.erase(start, end - start);
                            cursorPosition = start;
                        }
                        customPath.insert(cursorPosition, "    ");
                        cursorPosition += 4;
                    }
                    else if (event.text.unicode >= 32 && event.text.unicode < 128) {
                        if (selectionStart != selectionEnd) {
                            size_t start = std::min(selectionStart, selectionEnd);
                            size_t end = std::max(selectionStart, selectionEnd);
                            customPath.erase(start, end - start);
                            cursorPosition = start;
                        }
                        customPath.insert(cursorPosition, 1, static_cast<char>(event.text.unicode));
                        cursorPosition++;
                    }
                    selectionStart = selectionEnd = cursorPosition;
                    updateInputBox();
                    undoHistory.push_back(customPath);
                    undoIndex = undoHistory.size();
                }
            }
            else if (event.type == sf::Event::KeyPressed && isInputActive) {
                if (event.key.control && event.key.shift) {
                    if (event.key.code == sf::Keyboard::Left) {
                        moveWordWise(false, true);
                    } else if (event.key.code == sf::Keyboard::Right) {
                        moveWordWise(true, true);
                    } else if (event.key.code == sf::Keyboard::Z) {
                        // Redo
                        if (undoIndex < undoHistory.size() - 1) {
                            undoIndex++;
                            customPath = undoHistory[undoIndex];
                            cursorPosition = customPath.length();
                            selectionStart = selectionEnd = cursorPosition;
                            updateInputBox();
                        }
                    }
                }
                else if (event.key.control) {
                    if (event.key.code == sf::Keyboard::V) {
                        std::string clipboardContent = sf::Clipboard::getString();
                        clipboardContent.erase(std::remove(clipboardContent.begin(), clipboardContent.end(), '\n'), clipboardContent.end());
                        if (selectionStart != selectionEnd) {
                            size_t start = std::min(selectionStart, selectionEnd);
                            size_t end = std::max(selectionStart, selectionEnd);
                            customPath.erase(start, end - start);
                            cursorPosition = start;
                        }
                        customPath.insert(cursorPosition, clipboardContent);
                        cursorPosition += clipboardContent.length();
                        selectionStart = selectionEnd = cursorPosition;
                        updateInputBox();
                        undoHistory.push_back(customPath);
                        undoIndex = undoHistory.size();
                    } else if (event.key.code == sf::Keyboard::C) {
                        if (selectionStart != selectionEnd) {
                            size_t start = std::min(selectionStart, selectionEnd);
                            size_t end = std::max(selectionStart, selectionEnd);
                            sf::Clipboard::setString(customPath.substr(start, end - start));
                        }
                    } else if (event.key.code == sf::Keyboard::X) {
                        if (selectionStart != selectionEnd) {
                            size_t start = std::min(selectionStart, selectionEnd);
                            size_t end = std::max(selectionStart, selectionEnd);
                            sf::Clipboard::setString(customPath.substr(start, end - start));
                            customPath.erase(start, end - start);
                            cursorPosition = start;
                            selectionStart = selectionEnd = cursorPosition;
                            updateInputBox();
                            undoHistory.push_back(customPath);
                            undoIndex = undoHistory.size();
                        }
                    } else if (event.key.code == sf::Keyboard::Z) {
                        // Undo
                        if (undoIndex > 0) {
                            undoIndex--;
                            customPath = undoHistory[undoIndex];
                            cursorPosition = customPath.length();
                            selectionStart = selectionEnd = cursorPosition;
                            updateInputBox();
                        }
                    } else if (event.key.code == sf::Keyboard::Y) {
                        // Redo
                        if (undoIndex < undoHistory.size() - 1) {
                            undoIndex++;
                            customPath = undoHistory[undoIndex];
                            cursorPosition = customPath.length();
                            selectionStart = selectionEnd = cursorPosition;
                            updateInputBox();
                        }
                    } else if (event.key.code == sf::Keyboard::A) {
                        // Select all
                        cursorPosition = customPath.length();
                        selectionStart = 0;
                        selectionEnd = cursorPosition;
                    } else if (event.key.code == sf::Keyboard::Left) {
                        moveWordWise(false);
                    } else if (event.key.code == sf::Keyboard::Right) {
                        moveWordWise(true);
                    } else if (event.key.code == sf::Keyboard::Home) {
                        cursorPosition = 0;
                        selectionStart = selectionEnd = cursorPosition;
                    } else if (event.key.code == sf::Keyboard::End) {
                        cursorPosition = customPath.length();
                        selectionStart = selectionEnd = cursorPosition;
                    }
                } else if (event.key.shift) {
                    if (event.key.code == sf::Keyboard::Left) {
                        if (cursorPosition > 0) {
                            cursorPosition--;
                            selectionEnd = cursorPosition;
                        }
                    } else if (event.key.code == sf::Keyboard::Right) {
                        if (cursorPosition < customPath.length()) {
                            cursorPosition++;
                            selectionEnd = cursorPosition;
                        }
                    } else if (event.key.code == sf::Keyboard::Home) {
                        cursorPosition = 0;
                        selectionEnd = cursorPosition;
                    } else if (event.key.code == sf::Keyboard::End) {
                        cursorPosition = customPath.length();
                        selectionEnd = cursorPosition;
                    } else if (event.key.code == sf::Keyboard::Tab) {
                        // Shift+Tab functionality (remove indentation)
                        if (cursorPosition >= 4 && customPath.substr(cursorPosition - 4, 4) == "    ") {
                            customPath.erase(cursorPosition - 4, 4);
                            cursorPosition -= 4;
                            selectionStart = selectionEnd = cursorPosition;
                            updateInputBox();
                            undoHistory.push_back(customPath);
                            undoIndex = undoHistory.size();
                        }
                    }
                } else {
                    switch (event.key.code) {
                        case sf::Keyboard::BackSpace:
                            if (!customPath.empty() && (cursorPosition > 0 || selectionStart != selectionEnd)) {
                                if (selectionStart != selectionEnd) {
                                    size_t start = std::min(selectionStart, selectionEnd);
                                    size_t end = std::max(selectionStart, selectionEnd);
                                    customPath.erase(start, end - start);
                                    cursorPosition = start;
                                } else if (cursorPosition > 0) {
                                    customPath.erase(--cursorPosition, 1);
                                }
                                selectionStart = selectionEnd = cursorPosition;
                                updateInputBox();
                                undoHistory.push_back(customPath);
                                undoIndex = undoHistory.size();
                            }
                            break;
//                        case sf::Keyboard::Delete:
//                            if (!customPath.empty() && cursorPosition < customPath.length()) {
//                                if (selectionStart != selectionEnd) {
//                                    size_t start = std::min(selectionStart, selectionEnd);
//                                    size_t end = std::max(selectionStart, selectionEnd);
//                                    customPath.erase(start, end - start);
//                                    cursorPosition = start;
//                                } else {
//                                    customPath.erase(cursorPosition, 1);
//                                }
//                                selectionStart = selectionEnd = cursorPosition;
//                                updateInputBox();
//                                undoHistory.push_back(customPath);
//                                undoIndex = undoHistory.size();
//                            }
//                            break;
                        case sf::Keyboard::Left:
                            if (cursorPosition > 0) {
                                cursorPosition--;
                                selectionStart = selectionEnd = cursorPosition;
                            }
                            break;
                        case sf::Keyboard::Right:
                            if (cursorPosition < customPath.length()) {
                                cursorPosition++;
                                selectionStart = selectionEnd = cursorPosition;
                            }
                            break;
                        case sf::Keyboard::Home:
                            cursorPosition = 0;
                            selectionStart = selectionEnd = cursorPosition;
                            break;
                        case sf::Keyboard::End:
                            cursorPosition = customPath.length();
                            selectionStart = selectionEnd = cursorPosition;
                            break;
                        default:
                            break;
                    }
                }
            }
        }

        dialog.clear(sf::Color::White);

        if (isInputActive) {
            inputBox.setOutlineColor(sf::Color::Blue);
            inputBox.setOutlineThickness(3);
        } else {
            inputBox.setOutlineColor(sf::Color::Black);
            inputBox.setOutlineThickness(2);
        }
        dialog.draw(inputBox);

        dialog.draw(customPathText);

        if (isInputActive && selectionStart != selectionEnd) {
            float selStartX = customPathText.getPosition().x;
            float selEndX = customPathText.getPosition().x;
            for (size_t i = 0; i < std::min(selectionStart, selectionEnd); ++i) {
                selStartX += customPathText.getFont()->getGlyph(customPath[i], customPathText.getCharacterSize(), false).advance;
            }
            for (size_t i = 0; i < std::max(selectionStart, selectionEnd); ++i) {
                selEndX += customPathText.getFont()->getGlyph(customPath[i], customPathText.getCharacterSize(), false).advance;
            }
            sf::RectangleShape selection(sf::Vector2f(selEndX - selStartX, customPathText.getCharacterSize()));
            selection.setPosition(selStartX, customPathText.getPosition().y);
            selection.setFillColor(sf::Color(100, 100, 255, 128));
            dialog.draw(selection);
        }

        if (isInputActive) {
            float cursorX = customPathText.getPosition().x;
            for (size_t i = 0; i < cursorPosition; ++i) {
                cursorX += customPathText.getFont()->getGlyph(customPath[i], customPathText.getCharacterSize(), false).advance;
            }
            float cursorHeight = customPathText.getCharacterSize();
            sf::RectangleShape cursor(sf::Vector2f(2, cursorHeight));
            cursor.setPosition(cursorX, customPathText.getPosition().y);
            cursor.setFillColor(sf::Color::Black);

            if (static_cast<int>(clock.getElapsedTime().asSeconds() * 2) % 2 == 0) {
                dialog.draw(cursor);
            }
        }

        useDefaultPath.draw(dialog);
        useCustomPath.draw(dialog);
        dialog.draw(customPathPrompt);

        dialog.display();

        if (clock.getElapsedTime().asSeconds() >= 1) {
            clock.restart();
        }
    }
}


/**
 * Exports the simulation results to a file.
 * @param path The path where the file should be saved.
 */
void SystemManager::exportToFile(const std::string& path, bool isDefaultPath) {
    std::filesystem::path fsPath;
    try {
        if (isDefaultPath) {
            fsPath = std::filesystem::absolute(path);
        } else {
            fsPath = std::filesystem::path(path);

            // Ensure the path is absolute
            if (!fsPath.is_absolute()) {
                throw std::filesystem::filesystem_error("Path is not absolute",
                                                        std::error_code());
            }

            // Ensure the absolute path is to a directory
            if (!std::filesystem::is_directory(fsPath)) {
                throw std::filesystem::filesystem_error("Path is not a directory",
                                                        std::error_code());
            }

            // Append default filename to directory path
            fsPath /= houseName + "-" + demangle(typeid(myAlgo).name());

            // Convert to absolute path
            fsPath = std::filesystem::absolute(fsPath);
        }
    } catch (const std::filesystem::filesystem_error& e) {
        showMessageBox("Error: Invalid path\n" + std::string(e.what()));
        return;
    }

    std::ofstream outputFile;
    outputFile.open(fsPath);
    if (outputFile) {
        finalizeOutputFile(outputFile, vacuumCleaner, house, *finished, *score, steps);
        outputFile.close();
        showMessageBox("Output file has been exported to:\n\n" + fsPath.string());
    } else {
        showMessageBox("Error: Unable to open file for writing at:\n\n" + fsPath.string());
    }
}



/**
 * Displays a message box with the given message.
 * The box automatically resizes to fit the message and maintains aspect ratio when resized.
 * @param message The message to display in the box.
 */
void SystemManager::showMessageBox(const std::string& message) {
    unsigned int fontSize = 20;  // Increased font size
    sf::Text messageText(message, font, fontSize);
    messageText.setFillColor(sf::Color::Black);

    // Calculate text dimensions
    sf::FloatRect textBounds = messageText.getLocalBounds();

    // Set window dimensions based on text size
    int windowWidth = std::max(500, static_cast<int>(textBounds.width + 80));
    int windowHeight = std::max(200, static_cast<int>(textBounds.height + 150));

    sf::RenderWindow messageBox(sf::VideoMode(windowWidth, windowHeight), "Message");
    sf::View view = messageBox.getDefaultView();

    // Center the text in the window
    messageText.setPosition((windowWidth - textBounds.width) / 2, 40);

    Button okButton(sf::Vector2f(windowWidth / 2 - 50, windowHeight - 60), sf::Vector2f(100, 40), "OK", font, fontSize);

    while (messageBox.isOpen()) {
        sf::Event event;
        while (messageBox.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                messageBox.close();
            } else if (event.type == sf::Event::MouseButtonPressed) {
                sf::Vector2f mousePos = messageBox.mapPixelToCoords(sf::Mouse::getPosition(messageBox));
                if (okButton.isMouseOver(mousePos)) {
                    messageBox.close();
                }
            }
        }

        messageBox.clear(sf::Color::White);
        messageBox.draw(messageText);
        okButton.draw(messageBox);
        messageBox.display();
    }
}
#endif
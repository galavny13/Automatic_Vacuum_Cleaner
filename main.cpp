#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <thread>
#include <mutex>
#include <future>
#include <chrono>
#include <queue>
#include <atomic>
#include <exception>
#include <dlfcn.h>
#include <algorithm>
#include "simulation/simulator.h"
#include "common/AlgorithmRegistrar.h"
#include "simulation/utils.h"

namespace fs = std::filesystem;

// Mutex for thread-safe access to the scores map
std::mutex scoresMutex;

// Global vector to store handles of loaded shared libraries
std::vector<std::pair<void*, std::string>> loadedHandles;

/**
 * Structure to hold parameters required to run a simulation.
 */
struct RunnableParams {
    std::string houseFile;               // Path to the house file
    std::string algoName;                // Name of the algorithm to run
    std::vector<std::vector<int>> houseMap;  // 2D vector representing the house map
    std::string currentHouseName;        // Name of the current house
    std::string houseName;               // Name of the house
    int maxSteps;                        // Maximum number of steps allowed
    int maxBatterySteps;                 // Maximum battery steps available
    int rows;                            // Number of rows in the house map
    int cols;                            // Number of columns in the house map
    int initialDirt;                     // Initial amount of dirt in the house
    std::pair<int, int> dockingStation;  // Position of the docking station
    size_t timeoutMs;                    // Timeout for the simulation in milliseconds
};

/**
 * Runs a single simulation with the provided algorithm and parameters.
 *
 * @param simulator The simulator object responsible for running the simulation.
 * @param algorithm The algorithm to run in the simulation.
 * @param params Parameters required to run the simulation.
 * @param scores A map to store the resulting scores from the simulation.
 * @param summaryOnly If true, only a summary of the simulation will be generated.
 * @param animationMode If true, the simulation will run in animation mode.
 */
void runSimulation(MySimulator& simulator, std::unique_ptr<AbstractAlgorithm> algorithm,
                   const RunnableParams& params,
                   std::map<std::pair<std::string, std::string>, int>& scores,
                   bool summaryOnly, bool animationMode) {
    try {
        if (params.timeoutMs == 0) { // Animation mode
            // Set the algorithm and run the simulation without a timeout
            simulator.setAlgorithm(*algorithm, summaryOnly, animationMode, params.houseMap, params.currentHouseName,
                                   params.houseName, params.maxSteps, params.maxBatterySteps, params.rows, params.cols, params.dockingStation, params.initialDirt);
            simulator.run();  // Run the simulation
            std::lock_guard<std::mutex> lock(scoresMutex); // Lock mutex to safely access the scores map
            int score = simulator.getScore();
            scores[{params.algoName, fs::path(params.houseFile).stem().string()}] = score;
        } else { // Normal mode with timeout
            std::promise<void> simulationPromise;
            std::future<void> simulationFuture = simulationPromise.get_future();
            std::atomic<bool> stopFlag(false);

            // Lambda function to run the simulation task
            auto simulationTask = [&]() {
                try {
                    simulator.setAlgorithm(*algorithm, summaryOnly, animationMode, params.houseMap, params.currentHouseName,
                                           params.houseName, params.maxSteps, params.maxBatterySteps, params.rows, params.cols, params.dockingStation, params.initialDirt);
                    simulator.run(stopFlag);  // Run the simulation with the stop flag
                    simulationPromise.set_value(); // Signal that the simulation completed successfully
                } catch (...) {
                    simulationPromise.set_exception(std::current_exception()); // Capture any exception
                }
            };

            // Start the simulation in a separate thread
            std::thread simulationThread(simulationTask);

            auto start = std::chrono::steady_clock::now(); // Record start time

            // Wait for the simulation to finish or timeout
            auto status = simulationFuture.wait_for(std::chrono::milliseconds(params.timeoutMs));

            auto end = std::chrono::steady_clock::now(); // Record end time
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

            std::exception_ptr threadException;

            if (status == std::future_status::timeout) {
                // Timeout occurred, stop the simulation
                stopFlag.store(true);  // Signal the simulation to stop
                int score = params.maxSteps * 2 + params.initialDirt * 300 + 2000; // Calculate penalty score for timeout
                std::lock_guard<std::mutex> lock(scoresMutex);
                scores[{params.algoName, fs::path(params.houseFile).stem().string()}] = score;
                std::string errorMsg = "Timeout occurred.";
                auto exceptionPtr = std::make_exception_ptr(std::runtime_error(errorMsg));
                handleSimulationError(params.currentHouseName, params.algoName, exceptionPtr);
            } else {
                // Simulation completed within the allowed time
                try {
                    simulationFuture.get(); // Re-throw any exception from the simulation thread
                    int score = simulator.getScore(); // Get the simulation score
                    std::lock_guard<std::mutex> lock(scoresMutex);
                    scores[{params.algoName, fs::path(params.houseFile).stem().string()}] = score;
                } catch (...) {
                    // Handle any exception that occurred during the simulation
                    handleSimulationError(params.currentHouseName, params.algoName, std::current_exception());
                    int score = params.maxSteps * 2 + params.initialDirt * 300 + 2000; // Calculate penalty score for exception
                    std::lock_guard<std::mutex> lock(scoresMutex);
                    scores[{params.algoName, fs::path(params.houseFile).stem().string()}] = score;
                }
            }

            if (simulationThread.joinable()) {
                simulationThread.join(); // Wait for the simulation thread to finish
            }

            // Re-throw any exception captured from the simulation thread
            if (threadException) {
                std::rethrow_exception(threadException);
            }
        }
    } catch (...) {
        std::exception_ptr eptr = std::current_exception(); // Capture the current exception
        handleSimulationError(params.currentHouseName, params.algoName, std::current_exception());
        int score = params.maxSteps * 2 + params.initialDirt * 300 + 2000; // Calculate penalty score for exception
        std::lock_guard<std::mutex> lock(scoresMutex);
        scores[{params.algoName, fs::path(params.houseFile).stem().string()}] = score;
    }
}

/**
 * Worker function that processes tasks from the queue.
 * This function is executed by multiple threads in parallel to run simulations concurrently.
 *
 * @param tasks Queue of tasks to be processed by the worker.
 * @param scores Map to store the resulting scores from each simulation.
 * @param summaryOnly If true, only a summary of the simulation will be generated.
 * @param animationMode If true, the simulation will run in animation mode.
 */
void worker(std::queue<RunnableParams>& tasks, std::map<std::pair<std::string, std::string>, int>& scores,
            bool summaryOnly, bool animationMode) {
    while (true) {
        RunnableParams params;
        {
            std::lock_guard<std::mutex> lock(scoresMutex); // Lock mutex to safely access the task queue
            if (tasks.empty()) {
                break; // Exit loop if there are no more tasks to process
            }
            params = tasks.front(); // Get the next task from the queue
            tasks.pop(); // Remove the task from the queue
        }

        MySimulator simulator; // Create a new simulator instance for this task
        auto& registrar = AlgorithmRegistrar::getAlgorithmRegistrar(); // Get the AlgorithmRegistrar singleton instance
        auto it = std::find_if(registrar.begin(), registrar.end(),
                               [&](const auto& entry) { return entry.name() == params.algoName; }); // Find the algorithm by name

        if (it == registrar.end()) {
            std::cerr << "Algorithm not found: " << params.algoName << std::endl; // Handle case where algorithm is not found
            continue;
        }

        auto algorithm = it->create(); // Create an instance of the algorithm
        if (!algorithm) {
            handleAlgorithmCreationError(params.algoName); // Handle algorithm creation error
            continue;
        }

        runSimulation(simulator, std::move(algorithm), params, scores, summaryOnly, animationMode); // Run the simulation
    }
}

/**
 * Main function that handles command-line arguments and orchestrates the simulation process.
 *
 * @param argc Number of command-line arguments
 * @param argv Array of command-line argument strings
 * @return 0 if the program runs successfully, non-zero otherwise
 */
int main(int argc, char** argv) {
    std::string housePath = ".";   // Default path for house files
    std::string algoPath = ".";    // Default path for algorithm files
    int numThreads = 10;           // Default number of threads to use
    bool summaryOnly = false;      // Flag to indicate if only a summary should be generated
    bool foundHousePath = false;   // Flag to track if house path was provided
    bool foundAlgoPath = false;    // Flag to track if algorithm path was provided

    // Parse command-line arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg.find("-house_path=") == 0) {
            housePath = arg.substr(12); // Extract the house path from the command-line argument
            foundHousePath = true;     // Mark that the house path was provided
        } else if (arg.find("-algo_path=") == 0) {
            algoPath = arg.substr(11); // Extract the algorithm path from the command-line argument
            foundAlgoPath = true;      // Mark that the algorithm path was provided
        } else if (arg.find("-num_threads=") == 0) {
            std::string threadStr = arg.substr(13); // Extract the number part
            if (!threadStr.empty() && std::all_of(threadStr.begin(), threadStr.end(), ::isdigit)) {
                numThreads = std::stoi(threadStr); // Convert to int if all characters are digits
            } else {
                // Handle the case where x is not a valid integer
                std::cerr << "Error: The value for -num_threads must be an integer. The default value will be used." << std::endl;
            }
        } else if (arg == "-summary_only") {
            summaryOnly = true; // Set flag to generate only a summary
        }
    }

    // If house path or algorithm path is not provided, try to find the project root
    if (!foundHousePath || !foundAlgoPath) {
        // Lambda function to find the project root directory
        auto findProjectRoot = []() -> std::string {
            auto currentPath = fs::current_path();
            while (currentPath.has_parent_path()) {
                if (fs::exists(currentPath / "CMakeLists.txt")) {
                    return currentPath.string(); // Return path if CMakeLists.txt is found
                }
                currentPath = currentPath.parent_path(); // Move up one directory level
            }
            throw std::runtime_error("Project root not found"); // Throw exception if project root is not found
        };

        // Use the project root as the path if it wasn't provided
        if (!foundHousePath) housePath = findProjectRoot();
        if (!foundAlgoPath) algoPath = findProjectRoot();
    }

    bool animationMode = false; // Flag to indicate if the simulation should run in animation mode

    // Prompt the user to choose whether to enable animation mode
    while (true) {
        std::cout << "Do you want to be able to run in animation mode? (y/n)\n";
        std::cout << "Note: Animation mode disables multithreading and runs simulations sequentially.\n";
        std::string line;
        std::getline(std::cin, line); // Read the user's input

        char response = line.empty() ? '\0' : line[0];

        if (line.size() == 1 && (response == 'y' || response == 'Y')) {
            animationMode = true;
            numThreads = 1;  // Force single-threaded execution for animation mode
            break;
        } else if (line.size() == 1 && (response == 'n' || response == 'N')) {
            break; // Exit the loop if the user chooses not to enable animation mode
        } else {
            std::cout << "Invalid answer. Please enter y or n.\n" << std::endl; // Handle invalid input
        }
    }

    // Find all house files in the specified house path
    std::vector<std::string> houseFiles = findFiles(housePath, ".house");
    loadAlgorithms(algoPath, loadedHandles); // Load all algorithms from the specified path

    std::map<std::pair<std::string, std::string>, int> scores; // Map to store scores for each (algorithm, house) pair
    std::map<std::string, bool> validRuns; // Map to track valid runs based on house files

    std::queue<RunnableParams> tasks; // Queue to hold tasks for worker threads

    // Prepare tasks for each house file and algorithm
    for (const auto& houseFile : houseFiles) {
        std::vector<std::vector<int>> houseMap;
        std::string currentHouseName, houseName;
        int maxSteps = 0, maxBatterySteps = 0, rows = 0, cols = 0, initialDirt = 0;
        std::pair<int, int> dockingStation;

        try {
            // Read the house file and extract relevant data
            readHouseFile(houseFile, houseMap, currentHouseName, houseName, maxSteps,
                          maxBatterySteps, rows, cols, dockingStation, initialDirt);
            validRuns[fs::path(houseFile).stem().string()] = true; // Mark the house file as valid

            // Create a task for each algorithm and push it to the queue
            for (const auto& algo : AlgorithmRegistrar::getAlgorithmRegistrar()) {
                // Set timeout to 0 for animation mode, otherwise 1ms per step
                size_t timeoutMs = animationMode ? 0 : maxSteps * 1000;
                RunnableParams params{
                        houseFile, algo.name(), houseMap, currentHouseName, houseName,
                        maxSteps, maxBatterySteps, rows, cols, initialDirt, dockingStation,
                        timeoutMs
                };
                tasks.push(params); // Add the task to the queue
            }
        } catch (const std::exception& e) {
            handleHouseFileError(houseFile, std::current_exception()); // Handle any errors while reading the house file
            validRuns[fs::path(houseFile).stem().string()] = false; // Mark the house file as invalid
        }
    }

    // Execute tasks in animation mode (single-threaded)
    if (animationMode) {
        while (!tasks.empty()) {
            worker(tasks, scores, summaryOnly, animationMode); // Run tasks sequentially
        }
    } else {
        // Execute tasks in parallel using multiple threads
        std::vector<std::thread> threads;
        for (int i = 0; i < numThreads; ++i) {
            threads.emplace_back(worker, std::ref(tasks), std::ref(scores), summaryOnly, animationMode);
        }

        // Wait for all threads to complete
        for (auto& t : threads) {
            t.join();
        }
    }

    // Generate a summary CSV file with the results
    generateSummary(houseFiles,
                    [&]{
                        std::vector<std::string> names;
                        for (const auto& algo : AlgorithmRegistrar::getAlgorithmRegistrar())
                            names.push_back(algo.name());
                        return names;
                    }(),
                    scores, validRuns);

    // Clear the AlgorithmRegistrar before closing the handles
    AlgorithmRegistrar::getAlgorithmRegistrar().clear();

    // Close all loaded shared library handles
    for (const auto& [handle, filename] : loadedHandles) {
        if (handle) {
            int close = dlclose(handle);
            if (close != 0) {
                std::cerr << "Error closing shared library: " << dlerror() << std::endl; // Handle errors during dlclose
            }
        }
    }

    std::cout << "Program finished successfully" << std::endl; // Indicate successful completion

    return 0; // Return success status
}

#include "utils.h"


/**
 * Reads the house layout from a file and initializes the simulation parameters.
 * @param houseFilePath The path to the house layout file.
 */
void readHouseFile(const std::string& houseFilePath, std::vector<std::vector<int>>& houseMap,
                   std::string& currentHouseName, std::string& houseName, int& maxSteps, int& maxBatterySteps,
                   int& rows, int& cols, std::pair<int, int>& dockingStation, int& initialDirt) {
    std::filesystem::path path(houseFilePath);
    currentHouseName = path.stem().string();  // This gets the filename without extension

    std::ifstream file(houseFilePath); // Open the input file
    if (!file) {
        throw std::runtime_error("Error opening file: " + houseFilePath);
    }

    std::string line;
    int numOfDockingStationsFound = 0;

    // Read and parse house name / description
    if (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') { // If we're on a linux machine, then we might have \r as a char at the end of a line.
            line.pop_back(); // Remove the trailing \r character
        }
        std::regex pattern1(R"(MaxSteps\s*=\s*(\d+))");
        std::regex pattern2(R"(MaxBattery\s*=\s*(\d+))");
        std::regex pattern3(R"(Rows\s*=\s*(\d+))");
        std::regex pattern4(R"(Cols\s*=\s*(\d+))");
        std::smatch match;
        if (std::regex_match(line, match, pattern1) || std::regex_match(line, match, pattern2) || std::regex_match(line, match, pattern3)
            || std::regex_match(line, match, pattern4)) {
            throw std::runtime_error("Missing house name line");
        } else {
            houseName = line;
        }
    }

    // Read and parse MaxSteps
    if (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') { // If we're on a linux machine, then we might have \r as a char at the end of a line.
            line.pop_back(); // Remove the trailing \r character
        }
        std::regex pattern(R"(MaxSteps\s*=\s*(\d+))");
        std::smatch match;
        if (std::regex_match(line, match, pattern)) {
            maxSteps = std::stoi(match[1]);
        } else {
            throw std::runtime_error("Invalid or missing MaxSteps line");
        }
    }

    // Read and parse MaxBattery
    if (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') { // If we're on a linux machine, then we might have \r as a char at the end of a line.
            line.pop_back(); // Remove the trailing \r character
        }
        std::regex pattern(R"(MaxBattery\s*=\s*(\d+))");
        std::smatch match;
        if (std::regex_match(line, match, pattern)) {
            maxBatterySteps = std::stoi(match[1]);
        } else {
            throw std::runtime_error("Invalid or missing MaxBattery line");
        }
    }

    // Read and parse Rows
    if (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') { // If we're on a linux machine, then we might have \r as a char at the end of a line.
            line.pop_back(); // Remove the trailing \r character
        }
        std::regex pattern(R"(Rows\s*=\s*(\d+))");
        std::smatch match;
        if (std::regex_match(line, match, pattern)) {
            rows = std::stoi(match[1]);
        } else {
            throw std::runtime_error("Invalid or missing Rows line");
        }
    }

    // Read and parse Cols
    if (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') { // If we're on a linux machine, then we might have \r as a char at the end of a line.
            line.pop_back(); // Remove the trailing \r character
        }
        std::regex pattern(R"(Cols\s*=\s*(\d+))");
        std::smatch match;
        if (std::regex_match(line, match, pattern)) {
            cols = std::stoi(match[1]);
        } else {
            throw std::runtime_error("Invalid or missing Cols line");
        }
    }

    houseMap.resize(rows, std::vector<int>(cols, 0)); // Initialize the house map

    // Read the house layout
    for (int i = 0; i < rows; ++i) {
        if (std::getline(file, line)) {
            if (!line.empty() && line.back() == '\r') { // If we're on a linux machine, then we might have \r as a char at the end of a line.
                line.pop_back(); // Remove the trailing \r character
            }
            // Fill in missing columns with spaces
            if (line.size() < cols) {
                line.append(cols - line.size(), ' ');
            } else if (line.size() > cols) {
                line = line.substr(0, cols);
            }

            for (int j = 0; j < cols; ++j) {
                char ch = line[j];
                if (ch == ' ' || ch == '0') {
                    houseMap[i][j] = 0; // No dirt
                } else if (ch == 'D') {
                    if (numOfDockingStationsFound > 0) {
                        throw std::runtime_error("Multiple docking stations found in the house layout");
                    }
                    houseMap[i][j] = 0; // Docking station is marked
                    dockingStation = {i, j};
                    numOfDockingStationsFound++;
                } else if (ch >= '1' && ch <= '9') {
                    houseMap[i][j] = ch - '0'; // Convert character to dirt level
                    initialDirt += ch - '0';
                } else if (ch == 'W') {
                    houseMap[i][j] = -1; // Wall
                } else {
                    houseMap[i][j] = 0; // Any other character is treated as empty space
                }
            }
        } else {
            // Fill in missing lines with spaces
            std::fill(houseMap[i].begin(), houseMap[i].end(), 0);
        }
    }

    // Handle cases with no docking stations
    if (numOfDockingStationsFound == 0) {
        throw std::runtime_error("No docking station found in the house layout");
    }
}


std::string demangle(const char* name) {
    int status = -4; // some arbitrary value to eliminate the compiler warning
    std::unique_ptr<char, void(*)(void*)> res {
            abi::__cxa_demangle(name, nullptr, nullptr, &status),
            std::free
    };
    return (status == 0) ? res.get() : name;
}


/**
 * Converts a Step enum to its corresponding char representation.
 *
 * @param step The Step enum to convert.
 * @return A char representing the step:
 *         "N", "E", "S", "W", "s", or "F".
 */
char stepToChar(Step step) {
    switch (step) {
        case Step::North:  return 'N';
        case Step::East:   return 'E';
        case Step::South:  return 'S';
        case Step::West:   return 'W';
        case Step::Stay:   return 's';
        case Step::Finish: return 'F';
        default:           return 'U'; // For any unexpected values - shouldn't happen
    }
}


/**
 * Calculate the score of a given simulation.
 */
int calculateScore(const std::shared_ptr<VacuumCleaner>& vacuumCleaner,
                   const std::shared_ptr<House>& house, bool finished, int maxSteps) {

    int numSteps = vacuumCleaner->getSteps();
    int dirtLeft = house->getTotalDirtRemaining();

    std::string status;
    // Write status of the program
    if (finished) {
        status = "FINISHED";
    } else if (vacuumCleaner->getCurrentBatteryLevel() == 0) {
        status = "DEAD";
    } else {
        status = "WORKING";
    }

    bool inDock = vacuumCleaner->getPosition() == std::make_pair(0, 0);

    // Calculate score
    int score;
    if (status == "DEAD") {
        score = maxSteps + dirtLeft * 300 + 2000;
    } else if (status == "FINISHED" && !inDock) {
        score = maxSteps + dirtLeft * 300 + 3000;
    } else {
        score = numSteps + dirtLeft * 300 + (inDock ? 0 : 1000);
    }

    return score;
}


/**
 * Writes the final results of the simulation to the output file.
 */
void finalizeOutputFile(std::ofstream& outputFile, const std::shared_ptr<VacuumCleaner>& vacuumCleaner,
                       const std::shared_ptr<House>& house, bool finished, int score,
                       const std::shared_ptr<std::vector<Step>>& steps, bool timeout) {
    int numSteps = vacuumCleaner->getSteps();
    int dirtLeft = house->getTotalDirtRemaining();

    outputFile << "NumSteps = " << numSteps << "\n"; // Write total number of steps to output file
    outputFile << "DirtLeft = " << dirtLeft << "\n"; // Write amount of dirt left in the house

    std::string status;
    // Write status of the program
    if (finished) {
        outputFile << "Status = FINISHED\n";
        status = "FINISHED";
    } else if (vacuumCleaner->getCurrentBatteryLevel() == 0) {
        outputFile << "Status = DEAD\n";
        status = "DEAD";
    } else {
        outputFile << "Status = WORKING\n";
        status = "WORKING";
    }

    bool inDock = vacuumCleaner->getPosition() == std::make_pair(0, 0);
    // Write whether the vacuum cleaner is in the docking station or not
    if (inDock) {
        outputFile << "InDock = TRUE\n";
    } else {
        outputFile << "InDock = FALSE\n";
    }

    outputFile << "Score = " << score << "\n"; // Write the score to output file

    // Join steps with commas and write to the output file
    std::vector<char> charSteps;

    // Resize charSteps to match the size of steps
    charSteps.resize(steps->size());

    // Use std::transform to apply stepToChar to each element in steps and store the result in charSteps
    std::transform(steps->begin(), steps->end(), charSteps.begin(), stepToChar);

    std::ostringstream oss;
    for (size_t i = 0; i < charSteps.size(); ++i) {
        if (i != 0) {
            oss << ",";
        }
        oss << charSteps[i];
    }

    if (timeout) {
        oss << ",T";
    }

    outputFile << "Steps:\n";
    outputFile << oss.str();
}


/**
 * Compares two strings in a "natural" order, considering both alphabetical and numerical order.
 * This ensures that "MyHouse2" comes before "MyHouse10" in sorting.
 *
 * @param a The first string to compare
 * @param b The second string to compare
 * @return true if a should come before b in natural order, false otherwise
 */
bool naturalCompare(const std::string& a, const std::string& b) {
    auto it1 = a.begin(), it2 = b.begin();
    while (it1 != a.end() && it2 != b.end()) {
        if (std::isdigit(*it1) && std::isdigit(*it2)) {
            // Compare numbers
            auto n1 = std::find_if_not(it1, a.end(), [](char c) { return std::isdigit(c); });
            auto n2 = std::find_if_not(it2, b.end(), [](char c) { return std::isdigit(c); });

            int num1 = std::stoi(std::string(it1, n1));
            int num2 = std::stoi(std::string(it2, n2));

            if (num1 != num2) return num1 < num2;

            it1 = n1;
            it2 = n2;
        } else {
            // Compare characters
            if (*it1 != *it2) return *it1 < *it2;
            ++it1;
            ++it2;
        }
    }
    return a.size() < b.size();
}

/**
 * Finds all files with a specific extension in a given directory and sorts them using natural order.
 *
 * @param path The directory path to search in
 * @param extension The file extension to look for (including the dot, e.g., ".house")
 * @return A vector of file paths, sorted in natural order
 */
std::vector<std::string> findFiles(const std::string& path, const std::string& extension) {
    std::vector<std::string> files;
    for (const auto& entry : std::filesystem::directory_iterator(path)) {
        if (entry.path().extension() == extension) {
            files.push_back(entry.path().string());
        }
    }
    // Sort the files using natural sorting
    std::sort(files.begin(), files.end(), naturalCompare);
    return files;
}

/**
 * Recursively finds all files with a specific extension in a directory and its subdirectories.
 *
 * @param path The directory path to search in.
 * @param extension The file extension to search for (including the dot, e.g., ".so").
 * @return A vector of strings containing the full paths of all matching files.
 */
std::vector<std::string> findFilesRecursive(const std::string& path, const std::string& extension) {
    std::vector<std::string> result;

    // Iterate through the directory and its subdirectories
    for (const auto& entry : fs::recursive_directory_iterator(path)) {
        // Check if the current entry is a regular file and has the specified extension
        if (fs::is_regular_file(entry) && entry.path().extension() == extension) {
            // Add the full path of the file to the result vector
            result.push_back(entry.path().string());
        }
    }

    return result;
}

/**
 * Loads algorithms from shared library files (.so) in the specified directory and its subdirectories.
 * Creates error files for any algorithms that fail to load or register properly.
 *
 * @param path The directory path containing the algorithm shared libraries.
 * @param loadedHandles A reference to the vector storing the handles of the loaded shared libraries
 *                      and their corresponding file names.
 */
void loadAlgorithms(const std::string& path, std::vector<std::pair<void*, std::string>>& loadedHandles) {
    // Force initialization of AlgorithmRegistrar
    AlgorithmRegistrar::getAlgorithmRegistrar();

    // Find all .so files in the given path and its subdirectories
    std::vector<std::string> algorithmFiles = findFilesRecursive(path, ".so");

    // Iterate through each found .so file
    for (const auto& file : algorithmFiles) {
        // Get the initial count of registered algorithms
        size_t initialCount = AlgorithmRegistrar::getAlgorithmRegistrar().count();

        // Attempt to load the shared library
        void* handle = dlopen(file.c_str(), RTLD_NOW | RTLD_GLOBAL);

        if (!handle) {
            // If loading fails, create an error file
            std::string baseFilename = fs::path(file).filename().string();
            std::ofstream errorFile(fs::path(file).parent_path() / (fs::path(file).stem().string() + ".error"));
            errorFile << "Error loading " << file << ": " << dlerror() << std::endl;
            errorFile.close();
            continue;  // Move to the next file
        }

        // Check if a new algorithm was registered
        if (AlgorithmRegistrar::getAlgorithmRegistrar().count() == initialCount) {
            // If no new algorithm was registered, create an error file
            std::string baseFilename = fs::path(file).filename().string();
            std::ofstream errorFile(fs::path(file).parent_path() / (fs::path(file).stem().string() + ".error"));
            errorFile << "Error: No new algorithm registered from " << file << std::endl;
            errorFile.close();
            dlclose(handle);  // Close the library handle
        } else {
            // If a new algorithm was registered successfully, add the handle to loadedHandles
            loadedHandles.emplace_back(handle, file);
        }
    }
}

/**
 * Unloads all dynamically loaded algorithms by closing their associated shared libraries.
 * This function iterates over all handles stored during the loading phase and calls `dlclose`
 * to properly release the resources. If an error occurs during the unloading process, it writes
 * the error message to an error file named after the shared library, without the `.so` extension.
 *
 * @param loadedHandles A reference to the vector storing the handles of the loaded shared libraries
 *                      and their corresponding file names.
 */
void unloadAlgorithms(std::vector<std::pair<void*, std::string>>& loadedHandles) {
    // Iterate over all loaded handles, each associated with a shared library and its file name
    for (const auto& [handle, file] : loadedHandles) {
        // Check if the handle is valid (not null)
        if (handle) {
            // Attempt to close the shared library using dlclose
            if (dlclose(handle) != 0) {
                // Retrieve the error message from dlerror
                const char* error = dlerror();
                // Create an error file with the same name as the shared library, but without the .so extension
                std::ofstream errorFile(file.substr(0, file.size() - 3) + ".error");
                // Write the error message to the error file
                errorFile << "Error unloading " << file << ": " << error << std::endl;
                // Close the error file
                errorFile.close();
            }
        }
    }
    // Clear the loadedHandles vector to avoid any potential double-free issues or dangling pointers
    loadedHandles.clear();
}


/**
 * Generates a summary CSV file with simulation results for all house-algorithm combinations.
 *
 * @param houses A vector of house names
 * @param algorithms A vector of algorithm names
 * @param scores A map containing scores for each house-algorithm pair
 */
void generateSummary(const std::vector<std::string>& houses,
                     const std::vector<std::string>& algorithms,
                     const std::map<std::pair<std::string, std::string>, int>& scores,
                     std::map<std::string, bool> validRuns) {
    std::ofstream summaryFile("summary.csv");
    summaryFile << "Algorithm\\House";
    for (const auto& house : houses) {
        if (validRuns[std::filesystem::path(house).stem().string()]) {
            summaryFile << "," << std::filesystem::path(house).stem().string();
        }
    }
    summaryFile << std::endl;

    for (const auto& algo : algorithms) {
        summaryFile << algo;
        for (const auto& house : houses) {
            if (validRuns[std::filesystem::path(house).stem().string()]) {
                auto it = scores.find({algo, std::filesystem::path(house).stem().string()});
                if (it != scores.end()) {
                    summaryFile << "," << it->second;
                } else {
                    summaryFile << ",N/A";
                }
            }
        }
        summaryFile << std::endl;
    }
}


/**
 * Handles errors that occur when reading house files.
 *
 * @param houseFile The name of the house file that caused the error.
 * @param eptr A pointer to the exception that was thrown.
 */
void handleHouseFileError(const std::string& houseFile, std::exception_ptr eptr) {
    try {
        if (eptr) {
            std::rethrow_exception(eptr);
        }
    } catch (const std::ios_base::failure& e) {
        // Handle I/O errors
        std::ofstream errorFile(fs::path(houseFile).stem().string() + ".error");
        errorFile << "I/O error reading house file: " << e.what() << std::endl;
        errorFile.close();
    } catch (const std::runtime_error& e) {
        // Handle runtime errors (like invalid house format)
        std::ofstream errorFile(fs::path(houseFile).stem().string() + ".error");
        errorFile << "Invalid format in house file: " << e.what() << std::endl;
        errorFile.close();
    } catch (const std::out_of_range& e) {
        // Handle out of range errors (like invalid numbers in the house file)
        std::ofstream errorFile(fs::path(houseFile).stem().string() + ".error");
        errorFile << "Out of range error in house file: " << e.what() << std::endl;
        errorFile.close();
    } catch (const std::exception& e) {
        // Handle any other standard exceptions
        std::ofstream errorFile(fs::path(houseFile).stem().string() + ".error");
        errorFile << "Error reading house file: " << e.what() << std::endl;
        errorFile.close();
    } catch (...) {
        // Handle any other unexpected errors
        std::ofstream errorFile(fs::path(houseFile).stem().string() + ".error");
        errorFile << "Unknown error occurred while reading house file" << std::endl;
        errorFile.close();
    }
}



/**
 * Handles errors that occur when creating algorithms.
 *
 * @param algoName The name of the algorithm that failed to be created.
 */
void handleAlgorithmCreationError(const std::string& algoName) {
    std::ofstream errorFile(algoName + ".error");
    errorFile << "Error creating algorithm" << std::endl;
    errorFile.close();
}


/**
 * Handles errors that occur during simulation.
 *
 * @param houseFile The name of the house file related to the error.
 * @param algoName The name of the algorithm being used when the error occurred.
 * @param eptr A pointer to the exception that was thrown.
 */
void handleSimulationError(const std::string& houseFile, const std::string& algoName, std::exception_ptr eptr) {
    // Determine the path of the error file
    fs::path errorFilePath = algoName + ".error";

    // Check if the file exists and is not empty
    bool addNewline = fs::exists(errorFilePath) && fs::file_size(errorFilePath) > 0;

    // Open the error file in append mode
    std::ofstream errorFile(errorFilePath, std::ios_base::app);

    if (errorFile.is_open()) {
        // Add a new line if the file is not empty
        if (addNewline) {
            errorFile << std::endl;
        }

        // Write the houseFile name into the error file. TODO: Make sure houseFile is only the name
        errorFile << "Error occurred in the following house file: " << fs::path(houseFile).stem().string() << std::endl;

        try {
            if (eptr) {
                std::rethrow_exception(eptr);
            }
        } catch (const std::runtime_error& e) {
            // Handle runtime errors (like invalid house format, algorithm errors, etc.)
            errorFile << "Runtime error in simulation: " << e.what() << std::endl;
        } catch (const std::logic_error& e) {
            // Handle logic errors (like programming errors in the algorithm)
            errorFile << "Logic error in simulation: " << e.what() << std::endl;
        } catch (const std::bad_alloc& e) {
            // Handle memory allocation errors
            errorFile << "Memory allocation error in simulation: " << e.what() << std::endl;
        } catch (const std::exception& e) {
            // Handle any other standard exceptions
            errorFile << "Error in simulation: " << e.what() << std::endl;
        } catch (...) {
            // Handle any other unexpected errors
            errorFile << "Unknown error occurred in simulation" << std::endl;
        }

        // Close the file
        errorFile.close();
    } else {
        // Handle the error if the file couldn't be opened
        std::cerr << "Error: Unable to open file " << algoName << ".error" << std::endl;
    }
}
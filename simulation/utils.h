#ifndef UTILS_H
#define UTILS_H

#include <typeinfo>
#include <cxxabi.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <regex>
#include <memory>
#include <utility>
#include <cstdio>
#include <optional>
#include <filesystem>
#include <algorithm>
#include <cctype>
#include <dlfcn.h>
#include <map>
#include <atomic>
#include "common/AbstractAlgorithm.h"
#include "VacuumCleaner.h"
#include "House.h"
#include "algo/MyAlgo.h"
#include "bonus/SystemManager.h"
#include "common/AlgorithmRegistrar.h"

namespace fs = std::filesystem;

// Reads the house layout from a file and initializes the simulation parameters.
void readHouseFile(const std::string& houseFilePath, std::vector<std::vector<int>>& houseMap,
                   std::string& currentHouseName, std::string& houseName, int& maxSteps, int& maxBatterySteps,
                   int& rows, int& cols, std::pair<int, int>& dockingStation, int& totalDirtRemaining);
std::string demangle(const char* name);
char stepToChar(Step step);
void finalizeOutputFile(std::ofstream& outputFile, const std::shared_ptr<VacuumCleaner>& vacuumCleaner,
                       const std::shared_ptr<House>& house, bool finished, int score,
                       const std::shared_ptr<std::vector<Step>>& steps, bool timeout=false);
bool naturalCompare(const std::string& a, const std::string& b);
std::vector<std::string> findFiles(const std::string& path, const std::string& extension);
void loadAlgorithms(const std::string& path, std::vector<std::pair<void*, std::string>>& loadedHandles);
int calculateScore(const std::shared_ptr<VacuumCleaner>& vacuumCleaner,
                   const std::shared_ptr<House>& house, bool finished, int maxSteps);
void generateSummary(const std::vector<std::string>& houses,
                     const std::vector<std::string>& algorithms,
                     const std::map<std::pair<std::string, std::string>, int>& scores,
                     std::map<std::string, bool> validRuns);
void handleHouseFileError(const std::string& houseFile, std::exception_ptr eptr);
void handleAlgorithmCreationError(const std::string& algoName);
void handleSimulationError(const std::string& houseFile, const std::string& algoName, std::exception_ptr eptr);

#endif //UTILS_H
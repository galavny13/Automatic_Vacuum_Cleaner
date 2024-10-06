#ifndef HOUSE_H
#define HOUSE_H

#include <fstream>
#include <vector>
#include <string>
#include <utility>
#include <stdexcept>
#include <array>
#include <algorithm>
#include "common/enums.h"

/**
 * The House class represents the house environment with a grid, a docking station,
 * and functions to interact with and modify the house state.
 */
class House {
public:
    House() : grid({{0}}), dockingStation({0, 0}), totalDirtRemaining(0) {} // Default constructor for house

    House(std::vector<std::vector<int>> g, std::pair<int, int> dockingStation); // Constructor that initializes the house with a grid and a docking station

    [[nodiscard]] int getDirtLevel(int x, int y) const; // Gets the dirt level at the specified coordinates

    void clean(int x, int y); // Cleans the dirt at the specified coordinates

    [[nodiscard]] int getTotalDirtRemaining() const; // Gets the total dirt remaining in the house

    [[nodiscard]] bool isWall(Direction d, int x, int y) const; // Checks if there is a wall in the specified direction from the given coordinates

    // ----------------------------------------------------- BONUS ----------------------------------------------------

    void unClean(int x, int y);

private:
    std::vector<std::vector<int>> grid; // The grid representing the house
    std::pair<int, int> dockingStation; // The coordinates of the docking station
    int totalDirtRemaining; // The total dirt remaining in the house

    // Converts the grid to a full matrix
    void convertToFullMatrix();

    // Finds the longest dimensions in the grid
    void findLongestDimensions(size_t& longestRow, size_t& longestColumn);

    // Checks if the first row contains walls
    bool checkFirstRow();

    // Checks if the last row contains walls
    bool checkLastRow();

    // Checks if the first column contains walls
    bool checkFirstCol();

    // Checks if the last column contains walls
    bool checkLastCol(size_t longestRow);
};

#endif // HOUSE_H
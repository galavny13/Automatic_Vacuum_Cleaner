#include "House.h"


/**
 * Constructor to initialize House object.
 * @param g - A reference to the grid that we will get as input from the input file (or the default grid).
 * @param dockingStation - The position of the docking station in the house.
 */
House::House(std::vector<std::vector<int>> g, std::pair<int, int> dockingStation) : grid(std::move(g)), dockingStation(std::move(dockingStation)){
    convertToFullMatrix(); // Surround the house with walls
    totalDirtRemaining = 0;
    for (const auto& row : grid) { // Find the number of dirty cells in the grid
        for (int cell : row) {
            if (cell != -1 && cell != 0) { // Dirty cells are cells that are not equal to 0 and -1
                totalDirtRemaining += cell;
            }
        }
    }
}


/**
 * Returns the dirt level at the specified mathematical axis position relative to the docking station.
 * Assumes x and y are valid and are within the house grid when thinking of it as a mathematical axis
 * with the docking station being (0,0).
 * In the mathematical axis: (x, y) represents x steps right and y steps up from the origin (0, 0).
 * Note - if x is negative then (x, y) represents |x| steps left from the origin (0,0), and if y is negative then (x, y)
 * represents |y| steps down from the origin (0,0).
 * In the grid: Translates the mathematical axis position to the grid position.
 * @param x - Number of steps right (or left if negative) from the docking station.
 * @param y - Number of steps up (or down if negative) from the docking station.
 * @return - Returns the dirt level at the corresponding grid position.
 */
int House::getDirtLevel(int x, int y) const {
    return grid[dockingStation.first - y][dockingStation.second + x];
}


/**
 * Cleans the dirt (decreases it by 1) at the specified mathematical axis position relative to the docking station,
 * only if there is dirt to clean. Assumes x and y are valid and are within the house grid when thinking of it as a
 * mathematical axis with the docking station being (0,0).
 * In the mathematical axis: (x, y) represents x steps right and y steps up from the origin (0, 0).
 * Note - if x is negative then (x, y) represents |x| steps left from the origin (0,0), and if y is negative then (x, y)
 * represents |y| steps down from the origin (0,0).
 * In the grid: Translates the mathematical axis position to the grid position.
 * @param x - Number of steps right (or left if negative) from the docking station.
 * @param y - Number of steps up (or down if negative) from the docking station.
 * @return - Returns the dirt level at the corresponding grid position.
 */
void House::clean(int x, int y) {
    if (grid[dockingStation.first - y][dockingStation.second + x] > 0) {
        grid[dockingStation.first - y][dockingStation.second + x]--;
        totalDirtRemaining--;
    }
}


/**
 * Finds the longest dimensions of the grid.
 * @param longestRow Reference to the variable to store the longest row size.
 * @param longestColumn Reference to the variable to store the longest column size.
 */
void House::findLongestDimensions(size_t& longestRow, size_t& longestColumn) {
    // Initialize longestRow to 0
    longestRow = 0;
    // Initialize longestColumn to 0
    longestColumn = 0;

    // Iterate through each row in the grid
    for (const auto& row : grid) {
        // Update longestRow if current row size is greater
        longestRow = std::max(longestRow, row.size());
    }

    // Iterate through each column index up to longestRow
    for (size_t col = 0; col < longestRow; ++col) {
        size_t currentColumnSize = 0;
        // Iterate through each row in the grid to find the maximum column size across all rows
        for (const auto& row : grid) {
            // If the current column index is within the bounds of the row, increment currentColumnSize
            if (col < row.size()) {
                ++currentColumnSize;
            }
        }
        // Update longestColumn with the maximum size found
        longestColumn = std::max(longestColumn, currentColumnSize);
    }
}


/**
 * This is a private helper method that checks if we need to add a new row at the top of grid in order to make sure the
 * grid is surrounded by walls.
 * @return - true if all the cells in the first row are walls, otherwise false.
 */
bool House::checkFirstRow() {
    for (const auto& cell : grid.front()) {
        if (cell != -1) return false;
    }
    return true;
}


/**
 * This is a private helper method that checks if we need to add a new row at the bottom of grid in order to make sure
 * the grid is surrounded by walls.
 * @return - true if all the cells in the first row are walls, otherwise false.
 */
bool House::checkLastRow() {
    for (const auto& cell : grid.back()) {
        if (cell != -1) return false;
    }
    return true;
}


/**
 * This is a private helper method that checks if we need to add a new column at the left side of grid in order to
 * make sure the grid is surrounded by walls.
 * @return - true if all the cells in the first column are walls, otherwise false.
 */
bool House::checkFirstCol() {
    for (const auto& row : grid) {
        if (!row.empty() && row[0] != -1) return false;
    }
    return true;
}


/**
 * This is a private helper method that checks if we need to add a new column at the right side of grid in order to
 * make sure the grid is surrounded by walls.
 * @param longestRow - The length of the longest row in the original grid.
 * @return - Returns true if all the cells in the column at index longestRow - 1 are walls. NOTE - If a row is shorter
 * than longestRow - 1 - we consider it as if there is a wall in that cell, so it will still return true if those are
 * the only cells where we don't have a wall. Otherwise, return false.
 */
bool House::checkLastCol(size_t longestRow) {
    size_t maxColumnIndex = longestRow - 1;
    for (const auto& row : grid) {
        if (row.size() > maxColumnIndex && row[maxColumnIndex] != -1) {
            return false;
        }
    }
    return true;
}


/**
 * This is a private helper method that converts the original grid (after parsing it from the input file and making the
 * necessary changes to the inputted grid in readHouseFile) to a full rectangular matrix, while adding padding where
 * needed and making sure the grid is surrounded by walls (meaning we might need to add a new row or column of padding
 * at the bottom, top, left or right of the grid if it's not properly padded). Let's look at an example.
 * This grid:
 * 2 X
 * X 0 2 4 5
 * 1 0
 *
 * Will be converted to this rectangular matrix:
 * X X X X X X X
 * X 2 X X X X X
 * X X 0 2 4 5 X
 * X 1 0 X X X X
 * X X X X X X X
 */
void House::convertToFullMatrix() {
    // Variable to store the longest row in grid
    size_t longestRow;
    // Variable to store the longest column in grid
    size_t longestCol;
    findLongestDimensions(longestRow, longestCol); // Find the longest row and column in grid

    // Determine if extra rows or columns are needed at the borders
    bool needsTopPadding = !checkFirstRow();
    bool needsBottomPadding = !checkLastRow();
    bool needsLeftPadding = !checkFirstCol();
    bool needsRightPadding = !checkLastCol(longestRow);

    // Calculate new dimensions
    size_t newRows = longestCol + (needsTopPadding ? 1 : 0) + (needsBottomPadding ? 1 : 0);
    size_t newCols = longestRow + (needsLeftPadding ? 1 : 0) + (needsRightPadding ? 1 : 0);

    // Create newGrid with new dimensions, initialized with -1
    std::vector<std::vector<int>> newGrid(newRows, std::vector<int>(newCols, -1));

    size_t lastRow = (needsBottomPadding ? newRows - 1 : newRows);
    size_t lastCol = (needsRightPadding ? newCols - 1 : newCols);
    size_t firstRow = (needsTopPadding ? 1 : 0);
    size_t firstCol = (needsLeftPadding ? 1 : 0);

    // Copy the original grid into the new grid with padding
    for (size_t i = firstRow; i < lastRow; ++i) {
        for (size_t j = firstCol; j < lastCol; ++j) {
            if (j - firstCol < grid[i - firstRow].size())
                newGrid[i][j] = grid[i - firstRow][j - firstCol];
            else { // Add padding if needed
                newGrid[i][j] = -1;
            }
        }
    }

    grid = newGrid;
    dockingStation = {dockingStation.first + firstRow, dockingStation.second + firstCol};
}


/**
 * @return - Returns the total amount of dirt left in the house.
 */
int House::getTotalDirtRemaining() const {
    return totalDirtRemaining;
}


/**
 * Checks if there is a wall in the specified direction from the given position.
 * Assumes x and y are valid and are within the house grid when thinking of it as a mathematical axis
 * with the docking station being (0,0).
 * @param d Direction to check.
 * @param x X-coordinate relative to the docking station.
 * @param y Y-coordinate relative to the docking station.
 * @return true if there is a wall in the specified direction, false otherwise.
 */
bool House::isWall(Direction d, int x, int y) const {
    unsigned long row = dockingStation.first - y;
    unsigned long col = dockingStation.second + x;

    switch (d) {
        case Direction::North: row--; break;
        case Direction::East: col++; break;
        case Direction::South: row++; break;
        case Direction::West: col--; break;
        default: break;
    }
    return grid[row][col] == -1;
}


// ----------------------------------------------------- BONUS ----------------------------------------------------

void House::unClean(int x, int y) {
    grid[dockingStation.first - y][dockingStation.second + x]++;
    totalDirtRemaining++;
}
# Automatic Vacuum Cleaner Simulation

## Contributors
- Gal Avny
- Ben Cohen

## Summary
This project is an automatic vacuum cleaner simulation developed as part of a series of assignments for an advanced C++ programming course. The project evolved over three main assignments, where the complexity of both the algorithm and the simulation increased.

### Assignment 1

In the first assignment, we designed the foundational structure of the vacuum cleaner simulation. We created a simple vacuum cleaner that navigates through a house represented as a grid with dirt levels. The key features included:
- Reading a house layout from an input file, including walls, corridors, dirt levels, and a single docking station.
- Managing the vacuum cleaner's movement, battery life, and cleaning steps.
- Ensuring the vacuum can find its way back to the docking station for recharging before the battery depletes.

This initial implementation included basic movement, error handling, and the ability to output the number of steps and dirt left in the house at the end of the simulation. The algorithm was kept simple, focusing on ensuring that the vacuum does not move into walls or exceed the battery capacity.

### Assignment 2

Building upon the first assignment, assignment 2 introduced a more advanced algorithm and additional features:
- A structured house file format, which included rows and columns for the house layout, dirt levels, walls, and the docking station.
- The vacuum cleaner algorithm was upgraded to be "smart", utilizing search algorithms (e.g., BFS or DFS) to efficiently clean the house and return to the docking station with minimal steps.
- API for sensors such as dirt sensors, wall sensors, and battery meters was introduced, allowing the algorithm to make decisions based on environmental feedback.
- The output format was expanded to include the number of steps, dirt left, and the status of the vacuum (e.g., FINISHED, WORKING, DEAD).

This phase emphasized the interaction between the algorithm and the sensors, pushing the vacuum to make more efficient decisions based on the house layout.

### Assignment 3

The final assignment (Assignment 3) is where the full project came together. We were tasked with creating a full simulation framework and two separate algorithms for cleaning various house layouts. The key requirements for this assignment were:
- Creating two separate algorithms (submitted as shared object files, `.so`), each employing a different strategy for cleaning the house.
- Implementing a simulator that could load multiple house files and run the algorithms on each house, generating detailed output files.
- Adding multi-threading support to run multiple house-algorithm simulations simultaneously, with a timeout mechanism to ensure no simulation runs indefinitely.
- A scoring system based on the number of steps taken, the dirt left in the house, and whether the vacuum finished in the docking station or not.

The goal of the project was to create a robust, scalable solution that could handle multiple houses and algorithms while providing performance metrics for each run.

## Project Structure
```
project root
├── algo
│   ├── MyAlgo.h (abstract class that both algorithms inherit from)
│   ├── MyAlgo.cpp (abstract class that both algorithms inherit from)
│   ├── algo1
│   │   ├── algo_323855288_209648815_A.h
│   │   ├── algo_323855288_209648815_A.cpp
│   │   └── algo_323855288_209648815_A.so (after build)
│   └── algo2
│       ├── algo_323855288_209648815_B.h
│       ├── algo_323855288_209648815_B.cpp
│       └── algo_323855288_209648815_B.so (after build)
├── algorithm (part of the skeleton)
├── bonus
│   ├── bonus.txt (Animation explanation)
│   └── animation files (such as button.h, SystemManager.h and SystemManager.cpp)
├── common (part of the skeleton)
├── simulation
│   └── concrete sensors, house, vacuum cleaner, utils, and simulator
└── simulator (part of the skeleton)
```

### Bonus Features

As part of the project, we implemented several advanced features to enhance the simulation:

#### 1. Dynamic Visualization System:
- A real-time visualization system was developed using the SFML library to visualize the vacuum cleaner's movement in various house layouts. The system adapts to different house sizes, ensuring that the entire house remains visible during the simulation.

#### 2. Interactive Control Suite:
- Step-by-Step Navigation: Users can navigate through the simulation step by step, either forward or backward, allowing them to analyze each movement in detail.
- Pause/Resume: The simulation can be paused and resumed at any point.
- Variable Speed Control: Users can dynamically adjust the speed of the simulation based on their needs.

#### 3. End-of-Simulation Options:
- Result Export: After the simulation completes, users can export the results to a file for further analysis. 
- Custom Path Dialog: A custom text input system was developed to allow users to specify the path for saving results.

#### 4. Performance Optimization:
- Significant effort was invested in optimizing the rendering pipeline to ensure smooth animation and efficient memory management, even during long simulations or with large house layouts.

These features, while optional, greatly enhance the usability and performance of the simulation, offering both technical and user experience improvements.

## Algorithms

- Algorithm A: algo_323855288_209648815_A: Once the dirt is reached, the algorithm cleans it and repeats the scan to find the next closest dirty spot.
- Algorithm B: algo_323855288_209648815_B: If new dirt is discovered, the algorithm dynamically recalculates its path to optimize efficiency.


## Instructions for Running the Vacuum Cleaner Simulation

This program simulates a vacuum cleaner cleaning a house. It offers two modes: normal mode and animation mode.

**Note:** If running in animation mode, multithreading will not be available; simulations will run sequentially.

For each simulation:
- The algorithm and house file will be displayed on screen.
- You'll be prompted to choose between animation mode (1) or normal mode (2).


### Prerequisites

- C++ compiler (GCC recommended for Linux)
- CMake (version 3.10 or higher)
- SFML library (for animation mode)
- C++17 with std::filesystem support

### Installation of Prerequisites

#### 1. Installing CMake:
   For Linux (Ubuntu/Debian):\
   sudo apt update\
   sudo apt install cmake\
   <br>
   After installation, verify CMake is installed correctly:\
   cmake --version
   
#### 2. Installing SFML:
   For Linux (Ubuntu/Debian):\
   sudo apt-get update\
   sudo apt-get install libsfml-dev

#### 3. Ensuring C++17 or later with std::filesystem support:
   For Linux:
   - Ensure you have GCC 8+ or Clang 7+
   - When compiling, use the -std=c++17 flag (or -std=c++20 for C++20)

### Compilation and Execution

#### Automated Method (using scripts):

For Linux:
1. Ensure you have all the necessary files in your directory.
2. Open a terminal in the directory containing these files.
3. Make the script executable by running the following command:\
   chmod +x build_and_run.sh
4. Run the following command to configure, compile, and run the project:\
   ./build_and_run.sh -house_path=path/to/houses/directory -algo_path=path/to/algorithm/directory -num_threads=x -summary_only

**Note:** These scripts automate the process of configuring with CMake, building with make, and running the executable. They target all components of the project.

#### Manual Method (using CMake and make):

1. Create the build directory (if it doesn't exist. If it does - you might want to delete it before proceeding.):\
   mkdir -p cmake-build-debug\
   <br>
2. Configure the project with CMake:\
   cmake -B cmake-build-debug -S .\
   <br>
3. To build the entire project:\
   cmake --build cmake-build-debug\
   <br>
4. To build only a specific algorithm's shared library (e.g., MyAlgorithm):\
   cmake --build cmake-build-debug --target MyAlgorithm\
   <br>
5. To build only the myrobot executable:\
   cmake --build cmake-build-debug --target myrobot\
   <br>
6. Run the executable (from the project root directory):\
   ./myrobot -house_path=path/to/houses/directory -algo_path=path/to/algorithm/directory -num_threads=x -summary_only

#### Notes on Command-Line Arguments:
- If `num_threads` is not passed, it defaults to 10.
- `summary_only` is optional.
- If `house_path` is not passed, .house files are searched in the project root directory.
- If `algo_path` is not passed, .so files are searched in the project root directory.

#### Notes on the CMake, Makefile and script:
- The CMakeLists.txt file is configured to automatically target and compile all files in the `algo` directory, including those in nested subdirectories.
- MyAlgo.cpp is excluded from individual compilation as it's an abstract class, and therefore doesn't require its own .so file.
- The script uses a glob pattern to find all header files in the `algo` directory and its subdirectories.
- For each header file found (excluding MyAlgo.h and AlgorithmRegistration.h), the script checks for a corresponding .cpp file in the same directory.
- If a matching .cpp file exists, the `create_algorithm_library` function is called to create a shared library (.so file) for that algorithm.
- This automated process allows for easy addition of new algorithms without manually updating the CMakeLists.txt file.
- The MyAlgo.cpp file is included in the main executable compilation to ensure proper symbol resolution for derived classes.
- All generated .so files are placed in their respective algorithm directories within the `algo` folder.
- If the building with CMakeLists.txt fails, the script will try and fall back on the Makefile as backup.

Replace 'path/to/houses/directory' with the actual path to the directory containing your house layout files.\
Replace 'path/to/algorithm/directory' with the actual path to the directory containing your algorithm files.

The program will automatically detect if the animation mode is available:
- If SFML and filesystem support are available, the program will be able to run in animation mode.
- If not, the program will only run in normal mode without any additional input from the user.

**Note:** The CMake configuration script will check for the SFML and filesystem libraries. If they are not available,
the program will compile and run in normal mode without causing compilation errors.

#### Animation Mode Controls
- Space: Pause/Resume
- Right Arrow: Step forward (when paused)
- Left Arrow: Step backward (when paused)
- Speed Up/Down buttons: Adjust simulation speed

At the end of the simulation in animation mode, you can choose to export the results or quit the program.

**Notes:**
- The scripts will automatically configure and build the project using CMake.
- They check for the creation of `config.h` to ensure the configuration step was successful.
- The scripts will provide verbose output during both the configuration and build steps to help with debugging if something goes wrong.
- Make sure the paths to your house input files and algorithm files are correct and accessible by the script.

### Troubleshooting
- If you encounter issues with CMake not finding SFML, make sure you've installed it correctly and that it's in your system PATH.
- If you're having trouble with std::filesystem, make sure your compiler supports C++17 and that you're using the correct compilation flags.

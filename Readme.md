# Assignment 3 - Automatic Vacuum Cleaner Simulation

## Contributors
- Gal Avny 209648815
- Ben Cohen 323855288

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
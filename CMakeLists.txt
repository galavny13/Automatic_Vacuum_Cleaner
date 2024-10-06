cmake_minimum_required(VERSION 3.10)
project(CPP_HW3)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Try to find SFML package
find_package(SFML 2.5 COMPONENTS graphics window system QUIET)

# Check for std::filesystem support (manual check for C++17 support)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)

if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    check_cxx_source_compiles("
        #include <filesystem>
        int main() { return 0; }
    " HAS_FILESYSTEM)
else()
    message(FATAL_ERROR "The compiler does not support C++17")
endif()

# Create a config.h.in template if it doesn't exist
if(NOT EXISTS "${CMAKE_BINARY_DIR}/config.h.in")
    file(WRITE "${CMAKE_BINARY_DIR}/config.h.in" "#define USE_GUI @USE_GUI@\n")
endif()

# Set USE_GUI based on library availability
if(SFML_FOUND AND HAS_FILESYSTEM)
    set(USE_GUI 1)
else()
    set(USE_GUI 0)
endif()

configure_file("${CMAKE_BINARY_DIR}/config.h.in" "${CMAKE_BINARY_DIR}/config.h")

# Add definitions
add_definitions(-DUSE_GUI=${USE_GUI})

# Include directories
include_directories(
        "${CMAKE_BINARY_DIR}"
        "${CMAKE_SOURCE_DIR}"
)

# Function to create algorithm libraries
function(create_algorithm_library name dir)
    if(name STREQUAL "algo_323855288_209648815_A" OR name STREQUAL "algo_323855288_209648815_B")
        add_library(${name} SHARED
                algo/${dir}/${name}.cpp
                "algo/MyAlgo.cpp"  # Include MyAlgo.cpp only for these specific algorithms
        )
    else()
        add_library(${name} SHARED
                algo/${dir}/${name}.cpp
        )
    endif()
    target_include_directories(${name} PRIVATE
            "${CMAKE_BINARY_DIR}"
            "${CMAKE_SOURCE_DIR}"
    )
    set_target_properties(${name} PROPERTIES
            PREFIX ""
            SUFFIX ".so"
            LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/algo/${dir}"
    )
    target_link_libraries(${name} PRIVATE dl pthread)
endfunction()

# Add executable
add_executable(
        "main.cpp"
        "simulator/AlgorithmRegistrar.cpp"
        "simulation/simulator.cpp"
        "bonus/SystemManager.cpp"
        "simulation/VacuumCleaner.cpp"
        "simulation/ConcreteBatteryMeter.cpp"
        "simulation/ConcreteDirtSensor.cpp"
        "simulation/ConcreteWallsSensor.cpp"
        "simulation/House.cpp"
        "simulation/utils.cpp"
        "algo/MyAlgo.cpp"  # Include MyAlgo.cpp in the main executable for symbol resolution
)

# Set the output directory for myrobot to the project root
set_target_properties("myrobot" PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}"
)


# Add header files from algo directory and its subdirectories automatically
file(GLOB_RECURSE ALGORITHM_HEADERS "algo/**/*.h")

# Add header files
target_sources("myrobot" PRIVATE
        ${ALGORITHM_HEADERS}
        "common/AbstractAlgorithm.h"
        "common/BatteryMeter.h"
        "common/DirtSensor.h"
        "common/enums.h"
        "common/WallSensor.h"
        "simulation/simulator.h"
        "simulation/VacuumCleaner.h"
        "simulation/House.h"
        "common/AlgorithmRegistrar.h"
        "bonus/button.h"
        "bonus/SystemManager.h"
        "simulation/utils.h"
)

target_link_options("myrobot" PRIVATE -rdynamic)

target_link_libraries("myrobot" PRIVATE dl pthread)

if(USE_GUI)
    target_link_libraries("myrobot" PRIVATE sfml-graphics sfml-window sfml-system)
endif()

# Export symbols from the main executable
set_target_properties("myrobot" PROPERTIES ENABLE_EXPORTS ON)

# Create algorithm libraries
# Iterate over each file
foreach(header_file ${ALGORITHM_HEADERS})
    # Get the filename without the directory path
    get_filename_component(header_filename ${header_file} NAME)

    # Extract the base name (without extension)
    get_filename_component(header_basename ${header_filename} NAME_WE)

    # Skip MyAlgo.h and AlgorithmRegistration.h
    if(NOT header_basename STREQUAL "MyAlgo" AND NOT header_basename STREQUAL "AlgorithmRegistration")
        # Get the directory of the header file
        get_filename_component(header_dir ${header_file} DIRECTORY)
        get_filename_component(dir_name ${header_dir} NAME)
        # Check if corresponding .cpp file exists in the same directory as the header
        if(EXISTS "${header_dir}/${header_basename}.cpp")
            # Call create_algorithm_library for each valid file
            create_algorithm_library(${header_basename} ${dir_name})
        endif()
    endif()
endforeach()

message(STATUS "USE_GUI is set to ${USE_GUI}")
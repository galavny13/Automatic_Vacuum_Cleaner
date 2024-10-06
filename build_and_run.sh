#!/bin/bash

# Set the project directory
PROJECT_DIR=$(pwd)
BUILD_DIR="$PROJECT_DIR/cmake-build-debug"

# Parse command line arguments
HOUSE_PATH=""
ALGO_PATH=""
NUM_THREADS=""
SUMMARY_ONLY=""

while [[ $# -gt 0 ]]; do
    case $1 in
        -house_path=*)
        HOUSE_PATH="${1#*=}"
        shift
        ;;
        -algo_path=*)
        ALGO_PATH="${1#*=}"
        shift
        ;;
        -num_threads=*)
        NUM_THREADS="${1#*=}"
        shift
        ;;
        -summary_only)
        SUMMARY_ONLY="-summary_only"
        shift
        ;;
        *)
        # Unknown parameter, just shift and continue
        shift
        ;;
    esac
done

# Create or clean the build directory if it exists
if [ -d "$BUILD_DIR" ]; then
    echo "Cleaning existing build directory..."
    rm -rf "$BUILD_DIR"/*
else
    mkdir "$BUILD_DIR"
fi

# Change to the build directory
cd "$BUILD_DIR"

# Try to run CMake and build with SFML and std::filesystem support
cmake .. &> /dev/null
make &> /dev/null

# Check if the build succeeded
if [ $? -ne 0 ]; then
    echo "CMake build failed, attempting to build with Makefile..."

    # Change back to the project root directory
    cd "$PROJECT_DIR"

    # Run Makefile
    make &> /dev/null

    # Check if the build succeeded
    if [ $? -ne 0 ]; then
        echo "Makefile build failed. Please check your setup."
        exit 1
    else
        echo "Build succeeded with Makefile."
    fi
else
    echo "Build succeeded with CMake."
    # Change back to the project root directory
    cd "$PROJECT_DIR"
fi

# Construct the command to run the executable
CMD="./myrobot"

if [ ! -z "$HOUSE_PATH" ]; then
    CMD="$CMD -house_path=\"$HOUSE_PATH\""
fi

if [ ! -z "$ALGO_PATH" ]; then
    CMD="$CMD -algo_path=\"$ALGO_PATH\""
fi

if [ ! -z "$NUM_THREADS" ]; then
    CMD="$CMD -num_threads=$NUM_THREADS"
fi

if [ ! -z "$SUMMARY_ONLY" ]; then
    CMD="$CMD $SUMMARY_ONLY"
fi

# Run the executable
eval $CMD

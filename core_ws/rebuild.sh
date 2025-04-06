#!/bin/bash

# Remove the 'build' directory if it exists
if [ -d "build" ]; then
  echo "Removing existing build directory..."
  rm -rf build
fi

# Create a new 'build' directory
echo "Creating build directory..."
mkdir build

# Navigate into the 'build' directory
cd build

# Run cmake and make
echo "Running cmake .."
cmake -DBUILD_DEBUG=ON ..

echo "Running make"
make -j$(nproc)
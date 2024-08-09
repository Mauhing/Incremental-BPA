#!/bin/bash

# Step 1: Generate points using a Python script
echo "Generating points..."
python generate_points.py
echo "Points generated."

# Check if the build directory exists, if not create it
if [ ! -d "build" ]; then
    mkdir build
fi

# Step 2: Change directory to build, compile the project
echo "Building project..."
cd build
make
echo "Build complete."

# Step 3: Run the ballpivoting executable with parameters
echo "Running ballpivoting..."
./ballpivoting -i "../points_01.txt ../points_02.txt" -o ../mesh.ply -r "3"
echo "ballpivoting execution complete."

# Step 4: Run visualization Python script
echo "Visualizing the output..."
cd ..
python visualization.py
echo "Visualization complete."
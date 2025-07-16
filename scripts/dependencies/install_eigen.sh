#!/bin/bash

# Create deps directory if it doesn't exist
mkdir -p deps

# Download Eigen 3.4.0
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip -O deps/eigen.zip

# Unzip Eigen
cd deps
unzip -q eigen.zip

# Remove the zip file
rm eigen.zip

# Rename the extracted directory to 'eigen' for easier inclusion
if [ -d "eigen" ]; then
    rm -rf eigen
fi

echo "Eigen 3.4.0 installed successfully in deps/eigen" 
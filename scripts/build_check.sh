#!/bin/bash

# Build check script for AMR Fleet Adapter
# This script performs basic compilation checks

echo "=== AMR Fleet Adapter Build Check ==="

# Check if we're in the right directory
if [ ! -f "package.xml" ]; then
    echo "Error: package.xml not found. Please run from package root."
    exit 1
fi

# Check for required headers
echo "Checking header files..."
for header in include/amr_fleet_adapter/*.hpp; do
    if [ -f "$header" ]; then
        echo "✓ Found: $header"
    else
        echo "✗ Missing: $header"
    fi
done

# Check for source files
echo "Checking source files..."
for source in src/*.cpp; do
    if [ -f "$source" ]; then
        echo "✓ Found: $source"
    else
        echo "✗ Missing: $source"
    fi
done

# Check for configuration files
echo "Checking configuration files..."
for config in config/*.yaml config/*.json; do
    if [ -f "$config" ]; then
        echo "✓ Found: $config"
    else
        echo "✗ Missing: $config"
    fi
done

# Check for launch files
echo "Checking launch files..."
for launch in launch/*.py; do
    if [ -f "$launch" ]; then
        echo "✓ Found: $launch"
    else
        echo "✗ Missing: $launch"
    fi
done

echo ""
echo "=== Build Dependencies Check ==="
echo "Note: This package requires the following to build successfully:"
echo "- ROS2 Humble"
echo "- RMF packages (rmf_fleet_adapter, rmf_traffic, etc.)"
echo "- Nav2 packages" 
echo "- nlohmann-json-dev"
echo "- yaml-cpp"
echo "- OpenCV"
echo ""
echo "To build with ROS2:"
echo "1. Source ROS2: source /opt/ros/humble/setup.bash"
echo "2. Install dependencies: rosdep install --from-paths . --ignore-src -r -y"
echo "3. Build: colcon build --packages-select amr_fleet_adapter"
echo ""
echo "=== Build Check Complete ==="
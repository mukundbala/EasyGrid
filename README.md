![ROS 2 Jazzy CI](https://github.com/mukundbala/EasyGrid/actions/workflows/ros2_ci.yml/badge.svg?branch=main)
# EasyGrid
![EasyGrid Logo](docs/EasyGrid.png)
**A lightweight, header-only C++ library for managing 2D grids. Great for building occupancy, cost, or traversability maps for robotics applications in a single data structure!** 

## Motivation
2D Grids are instrumental to perception, navigation and a boatload of other tasks for mobile ground robots. As a result, I found myself rewriting grid-related functions and logic over and over again across different projects. This was ... extremely annoying. I wanted to fix this once and for all.

I wrote `EasyGrid` to create a single data structure to handle **several layers of grid information** while also keeping an **extensive set of helper methods** handy when dealing with grids. It is also header-only to make integration easy.

## Features
| Feature | Description |
|---------|-------------|
| **Coordinate Conversions** | Parent ↔ Grid ↔ Index conversions with SE(2)/SE(3) transform support |
| **Bounds Checking** | Methods to validate coordinates and indices against grid boundaries |
| **Cell Access** | Direct access and batch operations for reading/writing cell data |
| **Neighbor Queries** | Get 4-connected (cardinal) or 8-connected (octile) neighbors |
| **Point in Polygon** | Ray-casting algorithm with optional perimeter inclusion |
| **Cell Iterator** | Apply lambda functions to all cells or selected subsets |
| **Ray Iterator** | Bresenham line algorithm for tracing rays between points |
| **SubMap Iterator** | Iterate over rectangular regions of the grid |
| **Circle Iterators** | Visit cells on circle perimeter or within circle area |
| **Polygon Iterators** | Visit cells on polygon boundary or within polygon area |
| **Template-Based** | Flexible cell types - define any struct to store custom data |
| **Header-Only** | Single header implementation, easy integration |
| **Move Semantics** | Efficient memory management with move operations |
| **Spatial Awareness** | Full SE(3) transforms for grid positioning in parent frame |

## Demo
### RayIterator
![RayIterator Demo](docs/RayIterator.png)
---
### SubMapIterator
![SubMapIterator Demo](docs/SubMapIterator.png)
---
### CirclePerimeterIterator
![CirclePerimeterIterator Demo](docs/CirclePerimeterIterator.png)
---
### CircleAreaIterator
![CircleAreaIterator Demo](docs/CircleAreaIterator.png)
---
### PolygonPerimeterIterator
![PolygonPerimeterIterator Demo](docs/PolygonPerimeterIterator.png)
---
### PolygonAreaIterator
![PolygonAreaIterator Demo](docs/PolygonAreaIterator.png)
---

## Installing
### Install from source
```bash
mkdir -p grid_ws/src
cd grid_ws/src
git clone https://github.com/<github user>/EasyGrid.git
cd ..
colcon build --packages-up-to easy_grid
```

## Using EasyGrid in your project
Inside your `CmakeLists.txt` for a specific package to build EasyGrid against:
```CMake

find_package(easy_grid_core REQUIRED)
add_executable(your_ros_node
  src/your_ros_node.cpp
  src/your_code.cpp
)
target_include_directories(your_ros_node
    PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

target_link_libraries(your_ros_node easy_grid_core::easy_grid)
```
**See easy_grid_ros/CmakeLists.txt for an example**

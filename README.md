# OSM A* Planning

## Overview
This repository contains a Python implementation of the A* algorithm for path planning on OpenStreetMap (OSM) data.

## Features
- **A* Algorithm**: Implementation of the A* algorithm for finding the shortest path between two points on an OpenStreetMap graph.
- **OpenStreetMap Data**: Utilizes real-world map data from OpenStreetMap for path planning.
- **Customizable Heuristics**: Allows customization of heuristic functions for different pathfinding scenarios.
- **Visualization**: Includes tools to display the path found by the A* algorithm on the OpenStreetMap graph.

## Requirements
- Python3
- NetworkX
- osmnx
- Matplotlib

## Installation
```bash
   git clone https://github.com/AichaAbbad/osm_astar_planning.git
   ```

## Usage
1. Import the \`osm_astar_planning\` module into your Python project.
2. Instantiate an \`OSMAStarPlanner\` object.
3. Use the \`find_path\` method to find the shortest path between two points on the OpenStreetMap graph.
4. Visualize the path using the provided visualization tools.

Example:
```python
from osm_astar_planning import OSMAStarPlanner

# Instantiate OSMAStarPlanner object
planner = OSMAStarPlanner()

# Find path between two points
start_node = (latitude1, longitude1)
goal_node = (latitude2, longitude2)
path = planner.find_path(start_node, goal_node)

# Visualize the path
planner.visualize_path(path)
```

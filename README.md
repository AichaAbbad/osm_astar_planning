# OSM A* Planning

## Overview
This repository contains a Python implementation of the A* algorithm for path planning on OpenStreetMap (OSM) data.

Developed by: Aicha Manar ABBAD

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


## Example:
```python
from osm_astar_planning import OSMAStarPlanner

# Instantiate OSMAStarPlanner object
planner = Astar()

# Find path between two points
start_node = (latitude1, longitude1)
goal_node = (latitude2, longitude2)
path = planner.astar(start_node, goal_node)
```

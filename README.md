# Robot Path Planning - Project 1

This repository contains the implementation of the A* search algorithm that focuses on finding the lowest-cost path between two points, avoiding obstacles in a 2D grid.

## Project Overview

In this project, a robot must navigate from a start position to a goal position in a 2D workspace represented by an occupancy grid. The workspace includes both free cells (white) and obstacles (black). The robot can move in any of eight directions (horizontal, vertical, or diagonal), and the goal is to determine the path with the lowest cumulative cost.

### Cost Calculation

The cost for each move is calculated as the sum of:
- **Angle cost**: Penalizes changes in direction between consecutive moves.
- **Distance cost**: Depends on whether the move is horizontal/vertical (cost 1) or diagonal (cost √2).

## Features

- **A* Algorithm**: Utilizes an admissible heuristic based on Euclidean distance.
- **Graph Search**: Avoids revisiting states to ensure optimal pathfinding.
- **Configurable Angle Penalty**: Allows setting a penalty factor (`k`) for directional changes.

## Input and Output

### Input Format

- **Start and Goal Positions**: Coordinates in the format `(i, j)`.
- **Grid Layout**: 30x50 occupancy grid, where:
  - `0` - Free cell
  - `1` - Obstacle
  - `2` - Start position
  - `5` - Goal position

### Output Format

The program outputs a text file `final_output.txt` with 34 lines, including:
1. **Depth of Goal Node**: Level of the goal node.
2. **Total Nodes Generated**: Nodes generated in the search.
3. **Solution Path**: Sequence of moves from start to goal.
4. **Cost Values**: Costs for each move in the solution path.
5. **Final Workspace Layout**: Updated grid showing the path (cells marked as `4`).

## Running the Program
To execute the program, input the file path and optionally include the penalty factor `k`. If `k` is not supplied, it will be defaulted to 0.
 ```bash
 python RobotPathPlanning.py Sample_input.txt     # k defaults to 0
 python RobotPathPlanning.py Sample_input.txt 5   # Sets penalty factor k to 5
 ```
## Files

- `RobotPathPlanning.py`: Main program implementing the A* algorithm.
- `Sample_input.txt`: Example input file representing the workspace.
- `final_output.txt`: Output file showing the solution and workspace layout.

# README
# Software-Development-for-Intelligent-Autonomous-Vehicles
Software Development for Intelligent Autonomous Vehicles

## Overview
This project implements an **A\* pathfinding algorithm** for a grid-based environment using **Python** and **ROS (Robot Operating System)**. The algorithm calculates the shortest path between a start and goal point on a 2D grid while avoiding obstacles. The path is then published as a ROS topic for visualization or further use.

### Features
- Implementation of the A* algorithm for efficient pathfinding.
- Integration with ROS for real-time path publication.
- Customizable grid map for flexible pathfinding scenarios.

## Requirements

Before running the code, ensure you have the following dependencies installed:

- Python 3.x
- ROS (Robot Operating System, tested with ROS Noetic or newer)
- `numpy` library for grid management:
  ```bash
  pip install numpy
  ```

### ROS Dependencies
Make sure ROS is correctly installed and sourced.
- `nav_msgs` for publishing the path message.
- `geometry_msgs` for handling `PoseStamped` messages.

## Usage

1. **Grid Map**
   - The grid is defined in a 2D numpy array:
     - `0` represents a free cell.
     - `1` represents an obstacle.

   Example grid in the code:
   ```python
   grid = np.array([
       [0, 0, 0, 0, 0],
       [0, 1, 1, 1, 0],
       [0, 0, 0, 1, 0],
       [1, 1, 0, 1, 0],
       [0, 0, 0, 0, 0]
   ])
   ```

2. **Start and Goal Positions**
   - Define the `start` and `goal` coordinates:
   ```python
   start = (0, 0)
   goal = (4, 4)
   ```

3. **Running the Script**
   - Make sure to initialize a ROS node:
     ```bash
     roslaunch your_launch_file.launch
     ```
   - Run the Python script:
     ```bash
     python a_star_planner.py
     ```

4. **Output**
   - If a path is found, it will be published on the `/planned_path` topic.
   - The time taken to calculate the path will be displayed in the logs.

## Explanation of the Code

1. **A* Algorithm**
   - The `a_star` function calculates the shortest path from `start` to `goal`.
   - It uses a priority queue (`heapq`) to explore nodes in an optimal order.
   - The heuristic function is based on Manhattan distance:
   ```python
   def heuristic(a, b):
       return abs(a[0] - b[0]) + abs(a[1] - b[1])
   ```

2. **Publishing the Path**
   - The `publish_path` function takes the path and publishes it as a `Path` message for visualization in ROS.

3. **Main Execution**
   - The script initializes a ROS node named `a_star_planner`.
   - It logs the pathfinding process and publishes the result if a path is found.

## Example Output
```bash
[INFO] [Path found in 0.0012 seconds. Publishing...]
```
If no path is found:
```bash
[WARN] [No path found.]
```

## Notes
- Ensure that the grid boundaries and obstacles are set according to your environment.
- Adjust `start` and `goal` points as needed.

## Future Enhancements
- Implement dynamic obstacle handling.
- Add support for 3D environments.
- Incorporate additional heuristics for improved performance.

---

For any questions or improvements, feel free to contribute!



# RRT*-Connect

## for Single Query Path Planning and Obstacle Avoidance in Autonomous Vehicles

- The problem of finding a path efficiently to a goal while avoiding obstacles can be mapped to a <b>Search Problem</b> in Artificial Intelligence as we have to search for a path from a source to a destination.
- Our search space is the satellite-view image which is essentially a 2-D grid. The grid cells which are a part of any obstacle are blocked and we have to look for a path from the source to the destination through the unblocked cells.
- Hence, the search space can be reduced to a two-dimensional grid where some cells are blocked and some are not. The state information at each step consists of the cell (i,j) where we are currently at and the cells we have already covered during our search.
- We have explored search problems with just weights and edges and using the basic A* algorithm.
- <b>RRT* Connect</b> will have a better convergence rate and we will deal with the real life problem of obstacle avoidance in autonomous vehicles as well.

### Documentation

Read the report [here](https://docs.google.com/document/d/1x_x6wV92BNi2qEoeJY5fvgdGAckjnCtmiy7TnEU_wTw/edit#).

### Video Demo

Watch the video demo [here](https://www.youtube.com/watch?v=YUNbNW0-kWQ).

### Dependencies

1. `ROS Melodic`
2. `OpenCV v3.2.0`
3. `g++ 4.8 or higher`
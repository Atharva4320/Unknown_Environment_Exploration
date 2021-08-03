# Unknown_Environment_Exploration

![RViz final map in progress](https://user-images.githubusercontent.com/55175448/128098202-e77074d1-9d32-44f6-b3da-f847841b52c2.png)


This is the final project of RBE3002 course. The main goal of the project was to have the turtlebot3 autonomously navigate and map the unkown environment (in this case a maze). The entire project was done in a simulation using Rviz and programmed in Python. The team implemented A* Search Algorithm to find the optimal route to explore new areas in the maze (frontiers). Furthermore, the team implemented localization using ROS SLAM to improve the estimate of the current position. The entire project was divided into three parts:

### Part 1: Exploring the Maze
In this part the robot would be placed randomly at one corner of the entire maze and it has to navigate though the maze, collecting new information about its surroundings. This part ends when there are no more areas left to explore.

### Part 2: Returning to String Pose
Once the whole maze is explored, the robot then generates an updated version of the maze with the help of all the collected data. After creating the maze, the robot then returns to the starting position by finding the best route using the A* Search Algorithm.

### Part 3: User specified location
Once the robot reaches the starting position, the user can then choose a traversable point on the maze. The robot then travels to the user selected point using A* Search Algorithm.

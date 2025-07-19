# CS 480 - Vacuum Robot Planner
-Tenzin Konchok

This project implements a vacuum robot planner using Uniform-Cost Search (UCS) and Depth-First Search (DFS) algorithms to clean dirty cells in a grid world. All actions have a uniform cost.

## Make the world generator script executable and run it
chmod +x make_vacuum_world.py
./make_vacuum_world.py 5 7 0.15 3 > sample−5x7.txt
    This generates a 5-row by 7-column world with approximately 15% blocked cells and 3 dirty cells, and saves it to sample-5x7.txt.

## How to Run the Planner
python3 planner.py [algorithm] [world-file]
A* and BSF solver for Sokoban.
GUI is based on implementation from https://github.com/morenod/sokoban

Installation:
Install packages in requirements.txt file with pip or conda
1. pip install virtualenv
2. virtualenv venv
3. source venv/bin/activate
4. pip install -r requirements.txt

Instructions for running:

1.  cd to this directory
2.  python sokoban.py ASTAR (to run A*-Search)  
    python sokoban.py BFS (to run BFS)  
    python sokoban.py None (to play with arrows on keyboard without solver)
3.  Enter the number of a level (1 and 2 are recommended as the solution is found quite fast)
4.  Hit enter
5.  Press space for the found solution to execute

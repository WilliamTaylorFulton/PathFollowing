#!/usr/bin/env python

import Image, ImageDraw
import math
import time
import Queue
import sys
import os.path
import pickle
from discretemap import *

###################################
# BEGIN Global Variable Definitions
dmap = None  # This is our discrete map
# END Global Variable Definitions
###################################

#This class will be used in our search
class SearchNode:
    def __init__(self,state,parent,cost):
        self.parent = parent # Pointer to parent node (will be None for root node)
        self.state = state # The state (grid cell) this node points to
        self.cost = cost # The cumulative cost of this search node (sum of all actions to get here)
        
# This function returns the euclidean distance between two grid cells 
def euclidean_distance_grid(a,b):
    d = math.hypot(b[0]-a[0],b[1]-a[1])
    return d

# This function will set up everything we need for the search and run it
def run_a_star(start,goal):
    #start and goal are in grid coordinates

    #Create the start node
    start_node = SearchNode(start, None, 0)
    #Compute it's heuristic value
    start_node_h = euclidean_distance_grid(start,goal)

    #Create Fringe
    fringe = Queue.PriorityQueue()

    #Insert start node into fringe
    priority = start_node.cost + start_node_h # Compute it's f-value
    fringe.put((priority, start_node)) # Add it to the fringe

    print 'Starting A* search from grid cell ', start_node.state, ' to goal cell ', goal
    print 'Starting f-value is ', start_node_h + start_node.cost

    #Run the A* search
    goal_node = a_star(fringe,goal)

    #Extract path from the goal_node and return it    
    path = []

    if goal_node is not None:
        print 'Found a solution!'
        cur_node = goal_node
        while cur_node is not None:
            path.append(cur_node.state)
            cur_node = cur_node.parent
        path.reverse()

    return path
    
# This is the main A* function.  It performs the search
def a_star(fringe,goal):
    closed = [] # This keeps track of which grid cells we have already visited
    expansions = 0 # This keeps track of the number of expansions
    
    # Stay in loop as long as we have unexpanded nodes
    while not fringe.empty():
    
        # Get the node out of the fringe.  Format of items in the fringe is (priority, searchNode)
        expand_node = fringe.get()[1]  # This gets the node to exapadn
 
        # Make sure that we didn't already expand a node pointing to this state
        if expand_node.state not in closed:
            expansions = expansions + 1 # Increment our expansions counter
 
            #Test for the goal
            if expand_node.state == goal:
                # We found it!  Return the goal node.
                #  It stores the entire path through the parent pointers
                print 'Found the goal after ', expansions, ' node expansions'
                return expand_node
        
            # Add expanded node's id to the closed list
            closed.append(expand_node.state)

            # Add all this nodes neighbors that aren't already in the closed list
            neighbors = get_neighbors(expand_node.state, closed)

            #For each neighbor (a grid cell tuple(gx,gy) )
            for g in neighbors:
                #########################
                # Lab 5 Part B Code START
                #########################
                #Remove next line once you have code in the loop.  This is just so that python will run file as is
                print 'Exploring neighbor', g
                
                # 1. Compute the cost to get there from current node (use euclidean_distance_grid())
		cost = expand_node.cost + euclidean_distance_grid(expand_node.state, g)
                # 2. Create a new search node for it (constructor = SearchNode( grid_cell, parent SearchNode, cumulative cost)
		constructor = SearchNode(g, expand_node, cost)

                # 3. Compute the new node's heuristic value (use euclidean_distance_grid())
		new_hvalue = euclidean_distance_grid(constructor.state, goal)
                # 4. Compute the new node's priority (as per A* f-value calculation)
		priority = constructor.cost + new_hvalue
                # 5. Add it to the fringe, with proper priority (use fringe.put((priority, new SearchNode))
		fringe.put((priority, constructor))
                #########################
                # Lab 5 Part B Code END
                #########################

    # We have exhausted all possible paths.  No solution found
    print 'Didn\'t find a path after ', expansions, ' expansions'

# This function should return a list of the grid cells adjacent to
# g.  g = (x,y), so g[0] gives g's x-coordinate on grid, and g[1] gives g's y-coordinate on grid
# closed is a list of grid cells that have already been expanded
# dmap.occupied is a list of grid cells that have obstacles in them
# dmap.grid_width gives the width of the grid
# dmap.grid_height gives the height of the grid
def get_neighbors(g, closed):
    global dmap # Our discrete map
    neighbors = [] # The list of successors/neighbors

    #########################
    # Lab 5 Part A Code START
    #########################
    
    # Determine all of the neighbors/successors of grid cell g = (x,y)

    # Append each valid neighbor to g

    for y in (g[1]-1, g[1], g[1]+1):


	for x in (g[0]-1,g[0], g[0]+1):
		m = (x,y)
			
    # To be valid, neighbor must:
    #  1. Not already be in closed (list of already explored states)
    		if (m not in closed):
    #  2. Be on the map (check dmap.grid_width and dmap.grid_height)
   	 		if ((m[0] <= 0 or m[0] <= dmap.grid_width and m[1] <= 0 or m[1] <= dmap.grid_height)):

    #  3. Not be occupied by an obstacle (dmap.occupied is the list of occupied cells)
    				if (m not in dmap.occupied):
					neighbors.append(m)

    

    #########################
    # Lab 5 Part A Code END
    #########################

    # Return the list of neighbors
    return neighbors
    		 

if __name__ == '__main__':
    global dmap # The discrete map

    # Create the Discrete Map.  This will load start and goal locations from the world file
    dmap = DiscreteMap(sys.argv[1], 5)

    # Expand the obstacles by half the obstacle's radius (radius = 0.35)
    dmap.expand_obstacles(0.175)

    #Use A* to get the path from start to goal
    start = time.time()
    path = run_a_star(dmap.map_to_grid(dmap.start), dmap.map_to_grid(dmap.goal))
    end = time.time()
    duration = end - start

    #Uncomment to print out the path (in grid cell coordinates)
    #print 'PATH = ', path

    #Uncomment to print out the path (in map coordinates)
    #print 'MAP PATH = [',
    
    solution_quality = 0.0
    map_path = []
    for p in path:
        map_path.append(dmap.grid_to_map(p))
        if len(map_path) > 1:
            solution_quality += math.hypot((map_path[-1][0] - map_path[-2][0]),(map_path[-1][1] - map_path[-2][1]))
    #Uncomment to print out path in map coordinates
    #    print map_path[-1], ',',
    #Uncomment to print out path in map coordinates
    #print ']'

    #Print out statistics about this search
    print 'Quality/Cost of solution: ', solution_quality
    print 'Search took: ', duration, 'seconds'

    #Save an image of the path
    display_image_file = sys.argv[1][0:-6] + "_astarpath.png"
    dmap.display_path(path, display_image_file)
    print 'Saved image of path to file: ', display_image_file

    #Save the path to a file so that the waypoint_manager can load it later
    path_file_name = sys.argv[1][0:-6] + "_astarpath.pkl"
    path_file = open(path_file_name,'wb')
    pickle.dump(map_path,path_file)
    path_file.close()
    print 'Saved path to file: ', path_file_name

import sys
import heapq

class Node:
    def __init__(self, state, parent=None, action=None, cost=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.cost = cost

def reconstruct_path(goal_node):
    path = []
    current_node = goal_node
    while current_node.parent is not None:
        path.append(current_node.action)
        current_node = current_node.parent
    path.reverse()
    return path
    
def find_intial_state(grid, rows, cols):
    start_location = None
    initial_dirty_cells = set()

    for r in range(rows):
        for c in range(cols):
            cell_content = grid[r][c]

            if cell_content == '@':
                start_location = (r,c)
            elif cell_content == '*':
                initial_dirty_cells.add((r,c))

    if start_location is None:
        print("Error: Robot starting locaiton. '@' is not found in the grid!")
        sys.exit(1)

    return start_location, initial_dirty_cells


def get_successors(current_robot_loc, current_dirty_cells, game_grid, num_rows, num_cols):
    successors = []
    r, c = current_robot_loc
    
    #Dictionary
    moves = {
        'N': (r - 1, c),
        'S': (r + 1, c),
        'E': (r, c + 1),
        'W': (r, c - 1)
    }

    #current_dirty_cells set to a tuple of tuples
    #using sorted for consistent state representation
    current_dirty_cells_as_tuple = tuple(sorted(list(current_dirty_cells)))

    #check for moves
    for action, (next_r, next_c) in moves.items():
        #check boundary conditions
        if 0 <= next_r < num_rows and 0 <= next_c < num_cols:
            #check if the next cell is blocked
            if game_grid[next_r][next_c] != '#':
                # For move actions, dirty cells remain the same.
                # The new state is (new_robot_loc, same_dirty_cells_as_tuple)
                new_state = ((next_r, next_c), current_dirty_cells_as_tuple)
                successors.append((action, new_state))

    #checking for Vacuum action
    if current_robot_loc in current_dirty_cells:
        #creating a new set of dirty cells by removing the current location.
        new_dirty_cells_after_vacuum = current_dirty_cells.copy()
        new_dirty_cells_after_vacuum.remove(current_robot_loc)
        
        #converting new dirty cells set to a sorted tuple of tuples
        new_dirty_cells_after_vacuum_as_tuple = tuple(sorted(list(new_dirty_cells_after_vacuum)))

        #the new state for vacuum action
        new_state = (current_robot_loc, new_dirty_cells_after_vacuum_as_tuple)
        successors.append(('V', new_state))

    return successors

def is_goal_state(state):
    _robot_loc, dirty_cells_tuple = state
    return len(dirty_cells_tuple) == 0

def uniform_cost_search(initial_state, game_grid, num_rows, num_cols):
    #Returns:(list of actions, num_nodes_generated, num_nodes_expanded) 
    #else (None, 0, 0) if no solution is found.

    #Priority queue stores tuples: (cost, node_id, Node_object)
    #node_id to break ties if costs are equal
    frontier = []
    node_id_counter = 0

    initial_node = Node(initial_state)
    heapq.heappush(frontier, (initial_node.cost, node_id_counter, initial_node))
    node_id_counter += 1

    #dictionary: state_tuple -> min_cost
    cost_so_far = {initial_node.state: initial_node.cost}
    
    nodes_generated = 1 #initial node
    nodes_expanded = 0

    while frontier:
        #pop node with the lowest cost
        cost, _, current_node = heapq.heappop(frontier)
        nodes_expanded += 1

        #If found a cheaper path to this state, skip
        if cost > cost_so_far.get(current_node.state, float('inf')):
            continue

        #if this node's state is the goal
        if is_goal_state(current_node.state):
            path = reconstruct_path(current_node)
            return path, nodes_generated, nodes_expanded

        current_robot_loc, current_dirty_cells_tuple = current_node.state
        current_dirty_cells_set = set(current_dirty_cells_tuple)

        #generating successors
        for action, new_state_tuple in get_successors(current_robot_loc, current_dirty_cells_set, game_grid, num_rows, num_cols):
            new_cost = current_node.cost + 1 #Uniform cost of 1 for each action

            # If this path to new_state_tuple is cheaper or new
            if new_cost < cost_so_far.get(new_state_tuple, float('inf')):
                cost_so_far[new_state_tuple] = new_cost
                new_node = Node(state=new_state_tuple, parent=current_node, action=action, cost=new_cost)
                heapq.heappush(frontier, (new_node.cost, node_id_counter, new_node))
                node_id_counter += 1 # Increment for unique ID
                nodes_generated += 1

    return None, nodes_generated, nodes_expanded

def depth_first_search(initial_state, game_grid, num_rows, num_cols):
    #list as a LIFO stack:
    frontier = []
    
    #initial node has no parent, no action, and cost 0
    initial_node = Node(initial_state)
    frontier.append(initial_node)

    #visited states to avoid cycles.
    #set of states
    visited_states = set()
    visited_states.add(initial_node.state)

    nodes_generated = 1 #initial node generated
    nodes_expanded = 0

    while frontier:
        current_node = frontier.pop() #pop from end (LIFO)
        nodes_expanded += 1

        # Check if this node's state is the goal
        if is_goal_state(current_node.state):
            path = reconstruct_path(current_node)
            return path, nodes_generated, nodes_expanded

        current_robot_loc, current_dirty_cells_tuple = current_node.state
        
        #converting the dirty cells tuple back to a set for get_successors
        current_dirty_cells_set = set(current_dirty_cells_tuple) 

        # get_successors returns (action, new_state_tuple)
        for action, new_state_tuple in get_successors(current_robot_loc, current_dirty_cells_set, game_grid, num_rows, num_cols):
            #ckecking if this new state has not been visited
            if new_state_tuple not in visited_states:
                new_node = Node(state=new_state_tuple, parent=current_node, action=action, cost=current_node.cost + 1)
                frontier.append(new_node)
                visited_states.add(new_node.state) #mark the new state as visited
                nodes_generated += 1

    return None, nodes_generated, nodes_expanded

def parser(file):
    with open(file, "r") as f:
        cols = int(f.readline())
        rows = int(f.readline())
        
        grid = []

        for line in f:
            line = line.strip()
            current_row = list(line)
            grid.append(current_row)

    return cols,rows, grid
            
#MAIN
n = len(sys.argv)
algorithm = sys.argv[1]
word_file = sys.argv[2]

num_cols, num_rows, game_grid = parser(word_file)

initial_robot_loc, initial_dirty_cells_set = find_intial_state(game_grid, num_rows, num_cols)

#create initial state tuple (robot_loc_tuple, dirty_cells_tuple_of_tuples)
initial_dirty_cells_tuple = tuple(sorted(list(initial_dirty_cells_set)))
initial_state = (initial_robot_loc, initial_dirty_cells_tuple)

plan = None
nodes_generated = 0
nodes_expanded = 0

if algorithm == "uniform-cost":
    plan, nodes_generated, nodes_expanded = uniform_cost_search(initial_state, game_grid, num_rows, num_cols)
elif algorithm == "depth-first":
    plan, nodes_generated, nodes_expanded = depth_first_search(initial_state, game_grid, num_rows, num_cols)
else:
    print("Error: Invalid algorithm specified. Use 'uniform-cost' or 'depth-first'.")
    sys.exit(1)

#Results
if plan:
    for action in plan:
        print(action)
else:
    print("No solution found.")

print(f"{nodes_generated} nodes generated")
print(f"{nodes_expanded} nodes expanded")
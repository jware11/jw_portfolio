import streamlit as st
import numpy as np
from typing import List, Tuple, Dict, Callable
from copy import deepcopy

full_world = [
['🌾', '🌾', '🌾', '🌾', '🌾', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾'],
['🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌾', '🌾', '🌋', '🌋', '🌋', '🌋', '🌋', '🌋', '🌋', '🌾', '🌾'],
['🌾', '🌾', '🌾', '🌾', '🌋', '🌋', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌋', '🌋', '🌋', '⛰', '⛰', '⛰', '🌋', '🌋', '⛰', '⛰'],
['🌾', '🌾', '🌾', '🌾', '⛰', '🌋', '🌋', '🌋', '🌲', '🌲', '🌲', '🌲', '🐊', '🐊', '🌲', '🌲', '🌲', '🌲', '🌲', '🌾', '🌾', '⛰', '⛰', '🌋', '🌋', '⛰', '🌾'],
['🌾', '🌾', '🌾', '⛰', '⛰', '🌋', '🌋', '🌲', '🌲', '🌾', '🌾', '🐊', '🐊', '🐊', '🐊', '🌲', '🌲', '🌲', '🌾', '🌾', '🌾', '⛰', '🌋', '🌋', '🌋', '⛰', '🌾'],
['🌾', '⛰', '⛰', '⛰', '🌋', '🌋', '⛰', '⛰', '🌾', '🌾', '🌾', '🌾', '🐊', '🐊', '🐊', '🐊', '🐊', '🌾', '🌾', '🌾', '🌾', '🌾', '⛰', '🌋', '⛰', '🌾', '🌾'],
['🌾', '⛰', '⛰', '🌋', '🌋', '⛰', '⛰', '🌾', '🌾', '🌾', '🌾', '⛰', '🌋', '🌋', '🌋', '🐊', '🐊', '🐊', '🌾', '🌾', '🌾', '🌾', '🌾', '⛰', '🌾', '🌾', '🌾'],
['🌾', '🌾', '⛰', '⛰', '⛰', '⛰', '⛰', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '⛰', '🌋', '🌋', '🌋', '🐊', '🐊', '🐊', '🌾', '🌾', '⛰', '⛰', '⛰', '🌾', '🌾'],
['🌾', '🌾', '🌾', '⛰', '⛰', '⛰', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '⛰', '⛰', '🌋', '🌋', '🌾', '🐊', '🐊', '🌾', '🌾', '⛰', '⛰', '⛰', '🌾', '🌾', '🌾'],
['🌾', '🌾', '🌾', '🐊', '🐊', '🐊', '🌾', '🌾', '⛰', '⛰', '⛰', '🌋', '🌋', '🌋', '🌋', '🌾', '🌾', '🌾', '🐊', '🌾', '⛰', '⛰', '⛰', '🌾', '🌾', '🌾', '🌾'],
['🌾', '🌾', '🐊', '🐊', '🐊', '🐊', '🐊', '🌾', '⛰', '⛰', '🌋', '🌋', '🌋', '⛰', '🌾', '🌾', '🌾', '🌾', '🌾', '⛰', '🌋', '🌋', '🌋', '⛰', '🌾', '🌾', '🌾'],
['🌾', '🐊', '🐊', '🐊', '🐊', '🐊', '🌾', '🌾', '⛰', '🌋', '🌋', '⛰', '🌾', '🌾', '🌾', '🌾', '🐊', '🐊', '🌾', '🌾', '⛰', '🌋', '🌋', '⛰', '🌾', '🌾', '🌾'],
['🐊', '🐊', '🐊', '🐊', '🐊', '🌾', '🌾', '⛰', '⛰', '🌋', '🌋', '⛰', '🌾', '🐊', '🐊', '🐊', '🐊', '🌾', '🌾', '🌾', '⛰', '🌋', '⛰', '🌾', '🌾', '🌾', '🌾'],
['🌾', '🐊', '🐊', '🐊', '🐊', '🌾', '🌾', '⛰', '🌲', '🌲', '⛰', '🌾', '🌾', '🌾', '🌾', '🐊', '🐊', '🐊', '🐊', '🌾', '🌾', '⛰', '🌾', '🌾', '🌾', '🌾', '🌾'],
['🌾', '🌾', '🌾', '🌾', '🌋', '🌾', '🌾', '🌲', '🌲', '🌲', '🌲', '⛰', '⛰', '⛰', '⛰', '🌾', '🐊', '🐊', '🐊', '🌾', '🌾', '⛰', '🌋', '⛰', '🌾', '🌾', '🌾'],
['🌾', '🌾', '🌾', '🌋', '🌋', '🌋', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌋', '🌋', '🌋', '⛰', '⛰', '🌾', '🐊', '🌾', '⛰', '🌋', '🌋', '⛰', '🌾', '🌾', '🌾'],
['🌾', '🌾', '🌋', '🌋', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌋', '🌋', '🌋', '🌾', '🌾', '🌋', '🌋', '🌋', '🌾', '🌾', '🌾', '🌾', '🌾'],
['🌾', '🌾', '🌾', '🌋', '🌋', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌋', '🌋', '🌋', '🌋', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾'],
['🌾', '🌾', '🌾', '🌋', '🌋', '🌋', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌾', '🌾', '🌾', '⛰', '⛰', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾'],
['🌾', '🌾', '🌾', '🌾', '🌋', '🌋', '🌋', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🐊', '🐊', '🐊', '🐊'],
['🌾', '🌾', '⛰', '⛰', '⛰', '⛰', '🌋', '🌋', '🌲', '🌲', '🌲', '🌲', '🌲', '🌾', '🌋', '🌾', '🌾', '🌾', '🌾', '🌾', '🐊', '🐊', '🐊', '🐊', '🐊', '🐊', '🐊'],
['🌾', '🌾', '🌾', '🌾', '⛰', '⛰', '⛰', '🌋', '🌋', '🌋', '🌲', '🌲', '🌋', '🌋', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🐊', '🐊', '🐊', '🐊', '🐊', '🐊', '🐊'],
['🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '⛰', '⛰', '⛰', '🌋', '🌋', '🌋', '🌋', '🌾', '🌾', '🌾', '🌾', '⛰', '⛰', '🌾', '🌾', '🐊', '🐊', '🐊', '🐊', '🐊', '🐊'],
['🌾', '⛰', '⛰', '🌾', '🌾', '⛰', '⛰', '⛰', '⛰', '⛰', '🌾', '🌾', '🌾', '🌾', '🌾', '⛰', '⛰', '🌋', '🌋', '⛰', '⛰', '🌾', '🐊', '🐊', '🐊', '🐊', '🐊'],
['⛰', '🌋', '⛰', '⛰', '⛰', '⛰', '🌾', '🌾', '🌾', '🌾', '🌾', '🌋', '🌋', '🌋', '⛰', '⛰', '🌋', '🌋', '🌾', '🌋', '🌋', '⛰', '⛰', '🐊', '🐊', '🐊', '🐊'],
['⛰', '🌋', '🌋', '🌋', '⛰', '🌾', '🌾', '🌾', '🌾', '🌾', '⛰', '⛰', '🌋', '🌋', '🌋', '🌋', '⛰', '⛰', '⛰', '⛰', '🌋', '🌋', '🌋', '🐊', '🐊', '🐊', '🐊'],
['⛰', '⛰', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '⛰', '⛰', '⛰', '⛰', '⛰', '🌾', '🌾', '🌾', '🌾', '⛰', '⛰', '⛰', '🌾', '🌾', '🌾']
]

small_world = [
    ['🌾', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲'],
    ['🌾', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲'],
    ['🌾', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲'],
    ['🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾'],
    ['🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌾'],
    ['🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌾'],
    ['🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌾']
]

MOVES = [(0,-1), (1,0), (0,1), (-1,0)]

def heuristic(state: Tuple[int,int], goal: Tuple[int,int]):
    euc = np.sqrt((state[0] - goal[0])**2 + (state[1] - goal[1])**2)

    return euc

def get_neighbors(world: List[List[str]], current_state: Tuple[int, int], moves: List[Tuple[int, int]], 
                  costs: Dict[str, int]): 
    neighbors = []
    for move in moves: 
        x_move, y_move = move[0], move[1]
        test_x, test_y = current_state[0]+x_move, current_state[1]+y_move
        
        if (test_x < 0) or (test_y < 0): #out of bounds - negative
            continue
        if (test_x >= len(world)) or (test_y >= len(world[0])): #out of bounds
            continue
        if world[test_x][test_y] not in costs: #move not allowed
            continue

        neighbor = [test_x, test_y]
        neighbors.append(neighbor)
    
    return neighbors

def get_cost(world: List[List[str]], current_state: Tuple[int, int], costs: Dict[str,int]): 
    cur_x, cur_y = current_state[0], current_state[1]
    symbol = world[current_state[0]][current_state[1]]
    cost = costs[symbol]

    return cost

def on_list(neighbor: Tuple[int,int], list: List[Tuple[int,int]]): 
    found = False
    for state in list: 
        if state == neighbor: 
            found = True

    return found


def get_next_state(world: List[List[str]], frontier: List[Tuple[int,int]], costs: Dict[str, int], 
                   goal: Tuple[int, int], g: float):
    best_move = frontier[0]
    best_f = g + get_cost(world, best_move, costs) + heuristic(best_move, goal) #running cost + next cost + h
    
    for state in frontier: 
        test_f = g + get_cost(world, state, costs) + heuristic(state, goal)
        if test_f < best_f: #new best 
            best_move = state
            best_f = test_f

    return best_move

def get_direction(current: Tuple[int, int], next: Tuple[int, int]): 
    x1, y1 = current[0], current[1]
    x2, y2 = next[0], next[1]
    dy = x2 - x1 #translated for this world
    dx = y2 - y1 #translated for this world
    if dx == 0 and dy > 0: #pos y move
        return '⏬'
    if dx == 0 and dy < 0: #neg y move
        return '⏫'
    if dx > 0 and dy == 0: #pos x move
        return '⏩'
    if dx < 0 and dy == 0: #neg x move
        return '⏪'
    else: 
        return '❌'

def simple_path(path_dict: Dict, start: Tuple[int, int], goal: Tuple[int, int]):
    start = [start[0], start[1]] #convert to list
    goal = [goal[0], goal[1]] #convert to list
    cur_state, simple_path = goal, [goal]  #init
    
    while cur_state != start:
        prev_state = path_dict[str(cur_state)]['prev_pos']
        simple_path.insert(0, prev_state)
        cur_state = prev_state
    
    return simple_path

def a_star_search(world: List[List[str]], start: Tuple[int, int], goal: Tuple[int, int], costs: Dict[str, int], moves: List[Tuple[int, int]], heuristic: Callable) -> List[Tuple[int, int]]:
    start, goal = [start[0], start[1]], [goal[0], goal[1]] #convert to list
    frontier, explored, path_dict, g, current_state = [start], [], {}, 0, start #init
    while len(frontier) > 0: 
        prev_state = current_state #store
        current_state = get_next_state(world, frontier, costs, goal, g) #tuple position
        frontier.remove(current_state) #like pop
        cur_cost = get_cost(world, current_state, costs)
        g += cur_cost  #running cost        
        if (current_state[0] == goal[0]) and (current_state[1] == goal[1]): #we're done
            explored.append(current_state)            
            # return simple_path(path_dict, start, goal) #original return statement
            return simple_path(path_dict, start, goal), explored #added explored for st assignment
        neighbors = get_neighbors(world, current_state, moves, costs)
        for neighbor in neighbors: 
            if (not on_list(neighbor, frontier)) and (not on_list(neighbor, explored)) and (not neighbor == current_state) and (not neighbor == prev_state): #valid move
                frontier.append(neighbor)
                path_dict[str(neighbor)] = {'cost': get_cost(world, neighbor, costs), 'prev_pos': current_state}
        explored.append(current_state)
    return 'NO PATH FOUND' #path not found


def pretty_print_path(world: List[List[str]], path: List[Tuple[int, int]], start: Tuple[int, int], 
                      goal: Tuple[int, int], costs: Dict[str, int]) -> int:
    start = [start[0], start[1]] #convert to list
    goal = [goal[0], goal[1]] #convert to list
    total_cost = 0 #init
    cur_state = path[-1]  #init
    new_world = deepcopy(world) #copy
    for i in range(len(path)-1, 0, -1): 
        cur_state = path[i]
        total_cost += get_cost(world, cur_state, costs)
        if i != 0: 
            prev_state = path[i-1]
            dir_symbol = get_direction(prev_state, cur_state)
            new_world[prev_state[0]][prev_state[1]] = dir_symbol
    new_world[goal[0]][goal[1]] = '🎁' # goal symbol

    # for row in new_world: #original print code
    #     # print("".join(row))
    #     st.write("".join(row))
    return total_cost, new_world #added new_world output, suppressed print in this function

###########################################################################
###########################################################################
###########################################################################

# Changes to Original PA1 Code: 
# - a_star_search: adjusted return statement to also output 'explored''
# - pretty_print_path: adjusted return statement to include 'new_world'; commented out the print lines
# - Adjusted COSTS dictionary to allow for user-input
# - Adjusted start / goal variables to allow for user-input
# - All code below this point is new for the streamlit assignment (module04)


def show_explored(world: List[List[str]], explored: List[Tuple[int, int]], path: List[Tuple[int, int]], mode: str): 
    ## This function adds all explored states to a given world map using the defined symbol below
    symbol = '👀'
    exp_world = deepcopy(world)

    for item in explored: 
        if mode=='all' and item not in path: 
            exp_world[item[0]][item[1]] = symbol

        elif mode=='explored only': 
            exp_world[item[0]][item[1]] = symbol

    return exp_world

## Page Configuration
st.set_page_config(
    layout = 'wide',
    page_icon = '🏹', 
    page_title = 'jware11 A* Algorithm')

## init housekeeping
st.title('A* Search Algorithm')
st.subheader('jware11, Module 04, 18-February-2024')
st.write('This dashboard allows you to adjust inputs for the A* model, and view the effect on the overall path.')
st.write('*Use the Controls column to adjust start and goal states, along with costs for each obstacle.*')

## Generate session state to allow for resets
if "reset" not in st.session_state:
    st.session_state.reset = False

with st.container(): 
    col1, col2 = st.columns([0.2, 0.8]) #divide 20/80% for two columns

    defaults = {'plains':1, 'forest':3, 'hills':5, 'swamp':7, 'xs':0, 'ys':0} # default values used in resets

    ## Set Values for session state reset
    vplains = defaults['plains'] if st.session_state.reset else st.session_state.get('plains', defaults['plains'])
    vforest = defaults['forest'] if st.session_state.reset else st.session_state.get('forest', defaults['forest'])
    vhills = defaults['hills'] if st.session_state.reset else st.session_state.get('hills', defaults['hills'])
    vswamp = defaults['swamp'] if st.session_state.reset else st.session_state.get('swamp', defaults['swamp'])
    xs = defaults['xs'] if st.session_state.reset else st.session_state.get('xs', defaults['xs'])
    ys = defaults['ys'] if st.session_state.reset else st.session_state.get('ys', defaults['ys'])

    with col1:
        st.header("Controls")

        ## User Input Toggle Buttons
        toggle = st.toggle('Full World') #allow user to toggle small/full world
        toggle2 = st.toggle('Show Path') #allow user to toggle map vs path
        toggle3 = st.toggle('Show Explored') #allow user to show explored path

        if toggle: 
            max_x = len(full_world) - 1
            max_y = len(full_world[0]) - 1
        else: 
            max_x = len(small_world) - 1
            max_y = len(small_world) - 1

        ## User Inputs for Start and Goal states
        st.session_state.user_start_x = st.number_input('X Start', min_value=0, max_value=max_x, value=xs)
        st.session_state.user_start_y = st.number_input('Y Start', min_value=0, max_value=max_y, value=ys)
        user_goal_x = st.number_input('X Goal', min_value=0, max_value=max_x, value=max_x)
        user_goal_y = st.number_input('Y Goal', min_value=0, max_value=max_y, value=max_y)
        user_start = (st.session_state.user_start_x, st.session_state.user_start_y)
        user_goal = (user_goal_x, user_goal_y)

        ## Visual break
        st.write('-------')
        st.write('Costs:')

        ## User Inputs for costs
        st.session_state.plains = st.number_input('plains', min_value=1, max_value=10, value=vplains)
        st.session_state.forest = st.number_input('forest', min_value=1, max_value=10, value=vforest)
        st.session_state.hills = st.number_input('hills', min_value=1, max_value=10, value=vhills)
        st.session_state.swamp = st.number_input('swamp', min_value=1, max_value=10, value=vswamp)

        COSTS = { '🌾': st.session_state.plains, '🌲': st.session_state.forest, '⛰': st.session_state.hills, '🐊': st.session_state.swamp}

    if not toggle: #if small world
        with col2:
            st.header('Small World')
            small_start = user_start
            small_goal = user_goal
            small_path, explored = a_star_search(small_world, small_start, small_goal, COSTS, MOVES, heuristic)
            st.write('World Size: ', len(small_world), ',', len(small_world[0]))
            small_path_cost, path_world = pretty_print_path(small_world, small_path, small_start, small_goal, COSTS)

            if toggle2 and not toggle3: #show path, not explored
                for row in path_world:
                    st.write("".join(row))
                st.write(f"total path cost: {small_path_cost}")

            elif toggle2 and toggle3: #show path and explored
                exp_world = show_explored(path_world, explored, small_path, 'all')
                for row in exp_world: 
                    st.write("".join(row))
                st.write(f"total path cost: {small_path_cost}")

            elif not toggle2 and toggle3: #show explored only
                exp_world = show_explored(small_world, explored, None, 'explored only')
                for row in exp_world: 
                    st.write("".join(row))

            else: #show world
                for row in small_world: 
                    st.write("".join(row))

    else: # if full world
        with col2: 
            st.header('Full World')
            full_start = user_start
            full_goal = user_goal
            full_path, explored = a_star_search(full_world, full_start, full_goal, COSTS, MOVES, heuristic)
            st.write('World Size: ', len(full_world), ',', len(full_world[0]))
            full_path_cost, path_world = pretty_print_path(full_world, full_path, full_start, full_goal, COSTS)

            if toggle2 and not toggle3: #show path, not explored
                for row in path_world:
                    st.write("".join(row))
                st.write(f"total path cost: {full_path_cost}")

            elif toggle2 and toggle3: #show path and explored
                exp_world = show_explored(path_world, explored, full_path, 'all')
                for row in exp_world: 
                    st.write("".join(row))
                st.write(f"total path cost: {full_path_cost}")

            elif not toggle2 and toggle3: #show explored only
                exp_world = show_explored(full_world, explored, None, 'explored only')
                for row in exp_world: 
                    st.write("".join(row))

            else: #show world
                for row in full_world: 
                    st.write("".join(row))

    # Reset button
    if st.button("Reset values"):
        st.session_state.reset = True  # Mark reset as True if button is pressed
        st.experimental_rerun()  # Rerun the script
    else:
        st.session_state.reset = False  # Reset the reset flag


    ### Submission Notes ###
    # - This code is not the most efficient - in the future I would look to reduce total lines by re-using variable & function calls rather than repeating in loops
    # - This code uses several nested loops, which can be hard to follow. This seemed to be necessary to get the functionality I desired from toggle buttons and user inputs
    # - The 'reset' button doesn't always work as planned... sometimes it seems to reset only some variables, other times it resets all


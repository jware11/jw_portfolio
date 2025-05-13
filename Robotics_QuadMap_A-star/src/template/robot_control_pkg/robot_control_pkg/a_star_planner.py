import signal
import sys

import numpy as np
from typing import List, Tuple

from heapq import heappush, heappop


# class Edge: 
#     """
#     This class provides a basic data structure for representing
#     a directional edge in a graph. Travel is possible between
#     the starting node to the ending node at the given cost
#     but travel in the opposite direction is not allowed.
#     """
#     def __init__(self,starting_node, ending_node, cost):
#         self.start = starting_node
#         self.end = ending_node 
#         self.cost = cost 

#     def __repr__(self):
#         return 'Node'+self.__str__()
#     def __str__(self):
#         return f'({self.start.name},{self.end.name},{self.cost})'

#     def __eq__(self,obj):
#         if  isinstance(obj, Edge):
#             return self.start == obj.start and obj.end == obj.end and self.cost == self.cost 
#         return False
class GridNode:
    """
    This class provides a basic data structure for representing
    a node in A* Graph
    """
    def __init__(self, name, h):
        #The name of the node (can be anything, just for human readable output)
        self.name = name
        #The current best cost-to-come for the node
        self.g = np.inf 
        #The current best estimate of the node's total cost
        self.f = np.inf 
        #The heuristic estimate of the cost-to-go for the node
        self.h = h 
        #The list of edges which connect the node 
        self.edges = []
        #The previous node in path to the goal
        self.previous = None

    def add_neighbor(self, node, cost):
        new_edge = Edge(self, node, cost)
        self.edges.append(new_edge)

    def add_neighbor_bidirectional(self, node, cost):
        self.add_neighbor(node, cost)
        node.add_neighbor(self, cost)


    def __str__(self):
        return f'({self.name},{self.f},{self.g},{self.h})'

    def __eq__(self,obj):
        if  isinstance(obj, GridNode):
            return self.name == obj.name and self.f == self.f  and obj.g == obj.g and self.h == self.h 
        return False

    def __ge__(self, other):
        return self.f >= other.f

    def __lt__(self, other):
        return self.f < other.f

class A_Star:
    def __init__(self): 
        self.name = tester

    # def generate_adjacent(occmap: np.ndarray, node: GridNode, goal:GridNode):
    #     #from solution
    #     x = node.name[0]
    #     y = node.name[1]

    #     node_list = []
    #     for dx in range(-1,2,1):
    #         for dy in range(-1,2,1):
    #             # ignore the startingcell
    #             if (dx==0 and dy==0):
    #                 continue
    #             # Check if this cell beyond the map
    #             if (dx+x)<0 or (dy+y)<0:
    #                 continue
    #             # Check if this cell beyond the map
    #             if (dx+x)>=len(occmap) or (dy+y)>=len(occmap):
    #                 continue
    #             # Check if this cell is an obstacle
    #             if occmap[dx+x][dy+y] == 1:  
    #                 continue
    #             location = (x+dx, y+dy)
    #             node_list.append(GridNode(location, self.grid_dist(location,goal)))
    #     return node_list

    # def find_node(node_list: list, node):
    #     #from solution
    #     for n in node_list:
    #         if n.name == node.name:
    #             return n
    #     return None

    # def grid_dist(cell1,cell2):
    #     return np.sqrt((cell1[0]-cell2[0])**2+(cell1[1]-cell2[1])**2)

    def a_star_grid(map: np.ndarray, start:Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        This function will compute the optimal path between a start point and an end point given a grid-based
        map. It is up to the student to implement the heuristic function and cost function. Assume a cell's 
        indices represent it's position in cartesian space. (e.g. cells [1,3] and [1,5] are 2 units apart). 

        If no path exists then this function should return an empty list.

        Worth 50 pts
        
        Input
          :param map: An np.ndarray representing free space and occupied space
          :param start: A tuple of indicies indicating the starting cell of the search
          :param goal: A tuple of indicies indicating the goal cell of the search

        Output
          :return: path: a list of Tuples indicating the indicies of the cells that make up the path with 
                        the starting cell as the first element of the list and the ending cell as the last
                        element in the list
        """
        #moving functions inside
        def generate_adjacent(occmap: np.ndarray, node:GridNode, goal:GridNode):
            #from solution
            x = node.name[0]
            y = node.name[1]
            # print('node.name[0]: ', node.name[0])

            node_list = []
            for dx in range(-1,2,1):
                for dy in range(-1,2,1):
                    # ignore the startingcell
                    if (dx==0 and dy==0):
                        continue
                    # Check if this cell beyond the map
                    if (dx+x)<0 or (dy+y)<0:
                        continue
                    # Check if this cell beyond the map
                    if (dx+x)>=len(occmap) or (dy+y)>=len(occmap):
                        continue
                    # Check if this cell is an obstacle
                    # print('dx: ', dx)
                    # print('x: ', x)
                    # print('dx + x: ', dx+x)
                    if occmap[int(dx+x)][int(dy+y)] == 1:  
                        continue
                    location = (x+dx, y+dy)
                    node_list.append(GridNode(location, grid_dist(location,goal)))
            return node_list

        def find_node(node_list: list, node):
            #from solution
            for n in node_list:
                if n.name == node.name:
                    return n
            return None

        def grid_dist(cell1,cell2):
            return np.sqrt((cell1[0]-cell2[0])**2+(cell1[1]-cell2[1])**2)

        #from solution: 
        dist = grid_dist(start, goal)
        # begin = GridNode(start, self.grid_dist(start,goal))
        begin = GridNode(start, dist)
        end = GridNode(goal, 0.0)
        current = begin
        current.g = 0.0

        opened = [current]
        closed = []

        while len(opened) > 0:
            current = opened.pop(0)

            closed.append(current)

            if current == end:
                break

            adjacent = generate_adjacent(map, current, goal)

            for n in adjacent:
                if not find_node(closed, n):
                    n_old = find_node(opened, n)
                    if n_old: 
                        n = n_old
                    else:
                        opened.append(n)
                    edge_cost = grid_dist(current.name,n.name)
                    f = current.g + n.h + edge_cost
                    if f < n.f:
                        n.f = f
                        n.g = edge_cost + current.g
                        n.previous = current

            sorted(opened)

        # Trace the shortest path backward from the goal to the start
        path = []
        while not current == begin:
            path.insert(0, current.name)
            current = current.previous
        path.insert(0, current.name)

        #TODO Implement this function. 50 pts
        return path

    # def grid_demo():

    #     map = np.array([[0,0,1,0,0,0,0,0,0],
    #                     [0,0,1,0,0,0,0,0,0],
    #                     [0,0,1,0,0,1,1,1,0],
    #                     [0,0,1,0,0,1,0,0,0],
    #                     [0,0,1,0,0,1,0,1,1],
    #                     [0,0,1,0,0,1,0,0,0],
    #                     [0,0,0,0,0,1,0,0,0],
    #                     [0,0,0,0,0,1,0,0,0],
    #                     [0,0,0,0,0,1,0,0,0],
    #         ])

    #     start = (0,0)
    #     goal =  (8,8)
    #     path = a_star_grid(map, start, goal)
    #     for c_i in path:
    #         map[c_i[0], c_i[1]] = 5
    #         print(c_i)
    #     print(map)

    # def main():
    #     grid_demo()

    # if __name__ == '__main__':
    #     main()

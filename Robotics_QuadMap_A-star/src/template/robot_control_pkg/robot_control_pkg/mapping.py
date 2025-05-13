import signal
import sys

import numpy as np
from typing import List

from time import sleep

from matplotlib import pyplot
from matplotlib.patches import Rectangle

class MapType:
	UNKNOWN = -1
	UNOCCUPIED = 0
	OCCUPIED = 1

class QuadMap:
	"""
	This class maintains and updates a QuadTree representation of occupied and unoccupied space.
	It starts with all space being represented as unknown and updates the values of points in space
	based upon point_updates and ray_updates. 

	To implement this class students are provided with a framework for a QuadMapNode class. Using the
	provided QuadMapNode class is not necessary, students may create their own datastructure if they
	prefer. But the overall behavior of the QuadMap itself must be correct. 

	"""
	def __init__(self, 
				max_depth = 3,
				size = 5,
				origin = np.array([0.0,0.0])):
		"""
		Constructor for the QuadMap. All space is unknown when it is first initialized.

		The size of the quadmap represents its height and width.
		The origin represents the centerpoint of the quadmap.
		The default values should give a QuadMap that extends from [-5,5] at the bottom left 
		corner to [5,5] at the upper right corner. With a maximum resolution of size / 2^max_depth. 

		The constructor is graded on functionality, not a specific approach.

		Worth 10 pts

		Input
		  :param max_depth: The maximum depth of the quadtree.
		  :param size: The total height and width of the quadmap (i.e. the size of the root node)
		  :param origin: The centerpoint for the quadmap 
		"""

		self.root = QuadMapNode(parent=None, origin=origin, size=size)
		self.root.split() #initial split

		#self statements
		self.max_depth = max_depth
		self.size = size
		self.origin = origin

		#from solution
		self.depth = 0
		self.fig = None
		self.ray_step_size = self.size  / np.power(2, self.max_depth+1)

		
	def point_update(self, point: np.ndarray, state: int):
		"""
		Updates the value of the given point in the QuadMap. Update the value at the maximum depth.

		If the node located at this point already has this state do nothing.

		If the node located at this point is a leaf node at the maximum depth then update is value.

		If the node located at this point is a leaf node not at maximum depth then split it and repeat. 

		After updating the value of the point with its new state the quadmap should collapse any 
		nodes where all the children have the same value. 

		Worth 50 pts

		Input
		  :param point: A 2 element np.ndarray containing the x and y coordinates of the point
		  :param state: An integer indicating the MapType of the point
		"""
		#decompose
		(px, py) = (point[0], point[1])

		#print
		# print('point: ', point)
		# print('state: ', state)

		#initialize
		cur_node = self.root #start at origin node
		cur_depth = 0

		while (cur_depth < self.max_depth):
			if cur_node.state == state: 
				return
			if len(cur_node.children)==0: #no children
				cur_node.split() 
			
			cur_depth += 1
			id_offset = 0
			delta = abs(point - cur_node.origin)
			if delta[0] > cur_node.size / 2 or delta[1] > cur_node.size / 2:
				return
			if point[0] < cur_node.origin[0]:
				id_offset += 2
			if point[1] < cur_node.origin[1]: 
				id_offset += 1
			cur_node = cur_node.children[id_offset]

		#-- out of While - now at max depth
		assert cur_node.children == [] #at max depth, leaf node
		cur_node.state = state
		cur_node.recursive_combine()


	def ray_update(self, origin: np.ndarray, endpoint: np.ndarray):
		"""
		This function update the map based upon the return of a rangefinding beam.

		This function should update the map to indicate that all the space between the origin
		and the endpoint is unoccupied. Then update the endpoint as occupied.

		Worth 10 pts

		Input
		  :param origin: A 2 element ndarray indicating the location of the robot
		  :param endpoint: A 2 element ndarray indicating the location of the end of the beam
		"""
		# Decompose
		(xo, yo) = (origin[0], origin[1])
		(xe, ye) = (endpoint[0], endpoint[1])
		origin = ([xo, yo]) #reformat
		endpoint = ([xe, ye]) #reformat

		samples = 100 #changed from 1000
		xl = np.linspace(xo, xe, samples) 
		yl = np.linspace(yo, ye, samples) 
		points = [origin] #init

		for i in range(xl.size):
			temp = [xl[i], yl[i]]
			if not (temp == endpoint):
				toAdd = True
				for row in points: 
					if temp == row: 
						toAdd = False

				if toAdd == True: 
					points.append(temp)			

		for row in points: 
			self.point_update(row, 0) #unoccupied

		#Send endpoint
		self.point_update(endpoint, 1) #occupied


	def get_state(self, point: np.ndarray):
		#adapted from homework solution:

		cur_node = self.root
		while len(cur_node.children) > 0:
			id_offset = 0
			if point[0] < cur_node.origin[0]:
				id_offset += 2
			if point[1] < cur_node.origin[1]:
				id_offset += 1
			cur_node = cur_node.children[id_offset]
		return cur_node.state
	# 	cur_node = self.root
	# 	while cur_node.children != []: #while children to explore
	# 		Q = self.getQuad(cur_node, point)
	# 		cur_node = cur_node.children[Q-1]

	# 	#when cur_node.children is empty
	# 	return cur_node.state

	def to_occupancygrid2(self):	
			#converts to ros format, flattened

		size = np.int8(self.size)
		data = []
		for i in range(-size, size): #x traversal
			for j in range(-size, size): #y traversal
				point = ([j, i])
				temp = self.get_state(point)
				data.append(temp)

		data = (np.int8(data)).tolist()
		# data = data.tolist()

		# print('returning grid!')
		# print('grid type: ', type(data))
		# # print('grid size: ', data.size)
		# print('grid shape: ', len(data))
		# print(data)

		return data

	def to_occupancygrid(self):
		"""
		Converts the QuadMap into the data element of an occupancy grid. 
		A flattened representation used by ROS. 

		This is a 1-dimensional array with a length equal to the maximum number of nodes in the
		QuadMap. 

		Follow the guidance for the occupancy grid message
		https://github.com/ros2/common_interfaces/blob/master/nav_msgs/msg/OccupancyGrid.msg

		Worth 10 pts

		Output
		  :return: data: a list of integers representing the node values
		"""
		# max_nodes = 0
		# for i in range(self.max_depth): 
		# 	max_nodes += 4**i

		# size = np.int8(self.size)
		# data = []
		# for i in range(-size, size): #x traversal
		# 	for j in range(-size, size): #y traversal
		# 		point = ([j, i])
		# 		temp = self.get_state(point)
		# 		data.append(temp)

		# data = np.int8(data)

		# return data

		# from solution: 
		num_leaf = np.power(2, self.max_depth)
		grid = np.zeros((num_leaf,num_leaf))
		# print(grid,grid.shape)
		self.root.recursive_get_grid(grid)
		grid = np.int8(grid)
		# print('returning grid!')
		# print('grid type: ', type(grid))
		# print('grid size: ', grid.size)
		# print('grid shape: ', grid.shape)
		# print(grid)
		return grid

	def plot(self):
		"""
		A plot helper function. Much of the code is already in place but expects a working plot function for
		the QuadMapNode class. This function can be modified as necessary to work with your implementation.

		Worth 10 pts
		"""
		# Said this shouldn't be worth anything, so it should be done - don't break it and you get 10 points

		# fig = pyplot.figure(1, figsize=[5, 5], dpi=90)
		# ax = fig.add_subplot(111)

		# self.root.plot_node(ax)
		# pyplot.xlim([0, 10])
		# pyplot.ylim([0, 10])
		# pyplot.draw()
		# pyplot.pause(0.0001)

		#From solution below:

		if self.fig == None:
			self.fig, self.axis = pyplot.subplots(1,2,figsize=(15,5))

		grid = self.to_occupancygrid()
		pyplot.xlim([0, 10])
		pyplot.ylim([0, 10])
		self.root.plot_node(self.axis[1])
		self.axis[0].imshow(grid)

		pyplot.draw()
		pyplot.pause(0.0001)

class QuadMapNode:
	"""
	A partially implemented Node class for constructing a quadtree. 
	The student is welcome to modify this template implementation by adding
	new class variables or functions, or modifying existing variables and functions.

	The individual functions in QuadMapNode are not graded.

	Each QuadMapNode represents a node in a tree. Where every node other than the root
	has one parent and either 0 children or 4 children. Each node is half the length and width of
	its parent (1/4 the area). 
	"""
	def __init__(self, parent, origin: np.ndarray, size: float = None, state = MapType.UNKNOWN):
		"""
		Default constructor for the QuadMapNode.

		Input
		  :param parent: The parent node (if this node is root parent=None)
		  :param origin: A 2 element np.ndarray containing the centerpoint of the node
		  :param size: A float describing the height and width of the node

		"""
		if parent is None:
			self.parent = None
			assert size is not None
			self.size = size
		else:
			self.parent = parent
			self.size = parent.size / 2.0 

		self.origin = origin # The origin is the centerpoint of the node

		self.children = [] # A list of all child nodes. Should only contain 0 or 4 children.
		
		#self.state = MapType.UNKNOWN # A new node will not have a type assigned initially
		self.state = state


	# def split(self):
	# 	"""
	# 	Splits  node into 4 smaller nodes. This function should only be called if the current node is a leaf.

	# 	A useful function to implement for your quadmap
	# 	"""
	# 	(ox, oy) = (self.origin[0], self.origin[1])
	# 	if self.children == []: # leaf node
	# 		# inc = self.size / 2.0
	# 		inc = self.size / 4.0

	# 		#calc origins
	# 		O1 = np.array([ox + inc, oy + inc])
	# 		O2 = np.array([ox - inc, oy + inc])
	# 		O3 = np.array([ox - inc, oy - inc])
	# 		O4 = np.array([ox + inc, oy - inc])

	# 		#Create nodes
	# 		C1 = QuadMapNode(parent=self, origin=O1)
	# 		C2 = QuadMapNode(parent=self, origin=O2)
	# 		C3 = QuadMapNode(parent=self, origin=O3)
	# 		C4 = QuadMapNode(parent=self, origin=O4)

	# 		self.children = [C1, C2, C3, C4]

	# 	else: 
	# 		#Error - not a leaf node
	# 		print('Error: cannot split - not a leaf node!')

	def split(self):
		#FROM SOLUTION
		self.children.append(QuadMapNode(self, 
			self.origin+self.size/4*np.array([1.0,1.0]), self.size / 2.0, self.state))
		self.children.append(QuadMapNode(self, 
			self.origin+self.size/4*np.array([1.0,-1.0]), self.size / 2.0, self.state))
		self.children.append(QuadMapNode(self, 
			self.origin+self.size/4*np.array([-1.0,1.0]), self.size / 2.0, self.state))
		self.children.append(QuadMapNode(self, 
			self.origin+self.size/4*np.array([-1.0,-1.0]), self.size / 2.0, self.state))

	# def combine(self):
	# 	"""
	# 	Checks to see if all the children of the node have the same type, if they do then delete them and assign
	# 	the current node that type so that it becomes a leaf node.

	# 	A useful function to implement for your quadmap
	# 	"""
	# 	states = []
	# 	test = False
	# 	for i in self.children: 
	# 		states.append(i.state)

	# 	test = all(j == states[0] for j in states) 

	# 	if test and not (states == []): #all children states are same, set state, empty children
	# 		self.state = states[0]
	# 		self.children = []
	# 		# test == False #reset

	def combine(self): 
		'''
		Updated from solution
		'''
		if len(self.children) > 0:
			child_state = self.children[0].state
			for child in self.children:
				if child.state != child_state:
					return False
			self.children = []
			self.state = child_state
			return True
		return True

	def recursive_combine(self):
		#updated from solution
		if self.combine():
			self.parent.recursive_combine()
	

	def recursive_get_grid(self, ngrid: np.ndarray):
		"""
		FROM SOLUTION - used for occupancy grid

		Recursively slices the grid into smaller and smaller quadrants.
		Then sets the value of all cells in grid to the value of the leaf
		node. 

		This function takes advantage of the fact that python passes
		by reference to modify the larger array.

		Input
		  :param node: An instance of the QuadMapNode class that has a value and children
		  :param ngrid: An instance of an np.ndarray containing the slice of gridcells
		  				representing that node

		"""
		l = len(ngrid)
		l_half = int(l / 2)
		if self.children is None or len(self.children) == 0:
			# print(self.state)
			ngrid[:] = self.state
		else:
			self.children[0].recursive_get_grid(ngrid[:l_half,l_half:])
			self.children[1].recursive_get_grid(ngrid[l_half:,l_half:])
			self.children[2].recursive_get_grid(ngrid[:l_half,:l_half])
			self.children[3].recursive_get_grid(ngrid[l_half:,:l_half])


	def plot_node(self, ax):
		"""
		A helper function for visualizing the current state of the QuadMap. This function recursively calls itself
		against the children of the node. It may be necessary to modify this function to work with your implementation
		of the class.

		Note that only the leaf nodes are visualized!

		Input
		  :param ax: A matplotlib Axes object to plot the node
		"""
		if len(self.children) > 0:
			for child in self.children:
				child.plot_node(ax)
		else:
			if self.state == MapType.UNKNOWN:
				facecolor = 'grey'
			if self.state == MapType.UNOCCUPIED:
				facecolor = 'white'
			if self.state == MapType.OCCUPIED:
				facecolor = 'black'
			#print(self.origin,self.size,self.state)
			ax.add_patch(Rectangle((self.origin[0]-self.size/2,self.origin[1]-self.size/2),
				                   self.size, self.size,
				                   edgecolor = 'black',
				                   # edgecolor = facecolor, 
				                   facecolor = facecolor,
				                   fill=True,
				                   lw=5))

# def main():
    
# 	map = QuadMap(max_depth=5, size=10.0, origin=np.array([5.0,5.0]))

# 	x = np.array([2,6.5])
# 	ray = np.array([2,3])
# 	for i in range(50):
# 		map.ray_update(x, x+ray)
# 		map.plot()
# 		x+=np.array([0.025,-0.025])
# 		ray = np.array([[np.cos(0.1),-np.sin(0.1)],[np.sin(0.1),np.cos(0.1)]])@ray

# 	print('Done! Waiting 10 seconds then closing.')
# 	sleep(10)

# 	map.to_occupancygrid()

# if __name__ == '__main__':
#     main()

# Standard Algorithm Implementation
# Sampling-based Algorithms RRT and RRT*

import matplotlib.pyplot as plt
import numpy as np
from scipy import spatial
import sys
from networkx.classes.function import neighbors


# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.parent = None    # parent node
        self.cost = 0.0       # cost


# Class for RRT
class RRT:
    # Constructor
    def __init__(self, map_array, start, goal):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.start = Node(start[0], start[1]) # start node
        self.goal = Node(goal[0], goal[1])    # goal node
        self.vertices = []                    # list of nodes
        self.found = False                    # found flag


    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)

    
    def dis(self, node1, node2):
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        '''
        ### YOUR CODE HERE ###
        return spatial.distance.euclidean((node1.row, node1.col), 
                                          (node2.row, node2.col))

    
    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if the new node is valid to be connected
        '''
        ### YOUR CODE HERE ###
        pts = zip(np.linspace(node1.row, node2.row).astype(int), np.linspace(node1.col, node2.col).astype(int))

        for p in pts:
            if self.map_array[p[0], p[1]] == 0:
                return True
        return False


    def get_new_point(self, goal_bias):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point qrand=[row, col]
        '''
        ### YOUR CODE HERE ###
        # LOGIC: 
        # Create list of 10 elements with random size
        # Change the last element with goal
        # Create a probability list of 10 elements each representing its probability.. 
        # last elem will have goal_bias probability.. rest will have the remaining 1-goal_bias probabolity equally distributed
        sample_size = 10
        rng = np.random.default_rng()
        pos = rng.integers(0, self.size_row-1, (sample_size,2))
        pos[-1] = [self.goal.row, self.goal.col]
        prob = [(1-goal_bias)/(sample_size-1)]*sample_size
        prob[-1] = goal_bias
        # Use np.random.choice fn
        qrand_ind = (np.random.choice(range(sample_size), p=prob))
        qrand = pos[qrand_ind]
        
        return qrand

    
    def get_nearest_node(self, point):
        '''Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        '''
        ### YOUR CODE HERE ###
        qrand_node = Node(point[0], point[1]) 
        
        min = (None, sys.maxsize)     # Node, dist_from_goal
        for node in self.vertices:
            dist = self.dis(qrand_node, node)
            if dist < min[1]:
                min = (node, dist)  
            
        return min[0]


    def get_neighbors(self, new_node, neighbor_size):
        '''Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance 
        '''
        ### YOUR CODE HERE ###
        neighbors = []
        for node in self.vertices:
            dist = self.dis(new_node, node)
            if dist < neighbor_size:
                neighbors.append(node)
            
        return neighbors
    

    # Ref: https://www.youtube.com/watch?v=JM7kmWE8Gtc
    def rewire(self, new_node, neighbors):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''
        ### YOUR CODE HERE ###
        # Find the least cost parent for new_node and connect them
        min = (None, sys.maxsize)     # Node, dist_from_goal
        for node in neighbors:
            if self.check_collision(new_node, node) is True:
                continue
            cur_cost = node.cost + self.dis(new_node, node)
            if cur_cost < min[1]:
                min = (node, cur_cost)
        # if all neighbors are in collision, then chuck all of them
        if min[0] is not None:
            new_node.cost = min[1]
            new_node.parent = min[0]  
            self.vertices.append(new_node)
        
        # Now rewire the neighbors node through new_node if that provides a cheaper cost
        for node in neighbors:
            if self.check_collision(new_node, node) is True:
                continue
            cur_cost = new_node.cost + self.dis(new_node, node)
            if cur_cost < node.cost:
                node.parent = new_node
                node.cost = cur_cost
        
    
    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw Trees or Sample points
        for node in self.vertices[1:-1]:
            plt.plot(node.col, node.row, markersize=3, marker='o', color='y')
            plt.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')
        
        # Draw Final Path if found
        if self.found:
            cur = self.goal
            while cur.col != self.start.col or cur.row != self.start.row:
                plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='b')
                cur = cur.parent
                plt.plot(cur.col, cur.row, markersize=3, marker='o', color='b')

        # Draw start and goal
        plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='g')
        plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='r')

        # show image
        plt.show()      

        
    def smoothen_path(self):
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)
        
        # Draw Trees or Sample points
        for node in self.vertices[1:-1]:
            plt.plot(node.col, node.row, markersize=3, marker='o', color='y')
            plt.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')
        
        cur = self.goal
        mid2 = self.goal # slow tail node to find 33%
        mid1 = self.goal # slow tail node to find 66%
        i = 1
        while cur.col != self.start.col or cur.row != self.start.row:
            plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='b')
            cur = cur.parent
            plt.plot(cur.col, cur.row, markersize=3, marker='o', color='b')
            
            if i>1 and i%2==0:
                mid1 = mid1.parent
            elif i>1 and i%3==0:
                mid2 = mid2.parent
            i = i+1
                
        # Draw start and goal
        plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='g')
        plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='g')
        plt.plot(mid1.col, mid1.row, markersize=5, marker='o', color='g')
        plt.plot(mid2.col, mid2.row, markersize=5, marker='o', color='g')

        # Draw cubic Beizer curve 
        p0 = np.array([self.start.row, self.start.col])
        p1 = np.array([mid1.row, mid1.col])
        p2 = np.array([mid2.row, mid2.col])
        p3 = np.array([self.goal.row, self.goal.col])
        Cold_x = self.start.row
        Cold_y = self.start.col
        for t in np.linspace(0.1,1,20): 
            C_x = (1-t)**3*p0[0] + 3*(1-t)**2*t*p1[0] + 3*(1-t)*t**2*p2[0] + t**3*p3[0]
            C_y = (1-t)**3*p0[1] + 3*(1-t)**2*t*p1[1] + 3*(1-t)*t**2*p2[1] + t**3*p3[1]
            plt.plot([C_y, Cold_y], [C_x, Cold_x], color='r')
            plt.plot(C_y, C_x, markersize=10, marker='x', color='r')
            Cold_x = C_x
            Cold_y = C_y
            
        # show image
        plt.show()

    # Ref: https://journals.sagepub.com/doi/pdf/10.1177/0278364911406761
    def RRT(self, n_pts=1000):
        '''RRT main search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        In each step, extend a new node if possible, and check if reached the goal
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, check if reach the neighbor region of the goal.
        
        goal_region = 10
        goal_bias = 0.1
        nn_dist_threshold = 35
        # Stop when within the goal range or when max_itr reached
        itr = 0
        self.vertices.clear()
        self.vertices.append(self.start)
        while ( self.dis(self.vertices[-1], self.goal) >  goal_region or itr < n_pts ):
            itr = itr+1
            qrand = self.get_new_point(goal_bias)
            qrand_node = Node(qrand[0], qrand[1])
            q_nnode = self.get_nearest_node(qrand)         # q_nnode --> nearest node in tree
            
            dist = self.dis(q_nnode, qrand_node)
            if (dist < nn_dist_threshold):
                if self.check_collision(q_nnode, qrand_node) is False:
                    qrand_node.parent = q_nnode
                    qrand_node.cost = q_nnode.cost + dist
                    self.vertices.append(qrand_node)
            
        # set found flag
        q_nnode = self.vertices[-1]     # nearest node to goal
        if self.dis(q_nnode, self.goal) <=  goal_region:
            self.goal.parent = q_nnode
            self.goal.cost = q_nnode.cost + self.dis(q_nnode, self.goal)
            self.found = True

        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("----------------------%s------------------------" % "RRT")
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
            print("----------------------------------------------------------------")

        else:
            print("No path found")
        
        # Draw result
        # self.draw_map()
        self.smoothen_path()


    # Ref: https://journals.sagepub.com/doi/pdf/10.1177/0278364911406761
    def RRT_star(self, n_pts=1000, neighbor_size=20):
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            neighbor_size - the neighbor distance
        
        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, rewire the node and its neighbors,
        # and check if reach the neighbor region of the goal if the path is not found.
        
        goal_region = 15
        goal_bias = 0.10
        nn_dist_threshold = 40
        neighbor_size = 40
        # Don't stop when within the goal range, but keep trying for most optimal path till max_itr reached
        itr = 0
        self.vertices.clear()
        self.vertices.append(self.start)
        while ( itr < n_pts ):
            itr = itr+1
            qrand = self.get_new_point(goal_bias)
            qrand_node = Node(qrand[0], qrand[1])
            q_nnode = self.get_nearest_node(qrand)         # q_nnode --> nearest node in tree
            
            dist = self.dis(q_nnode, qrand_node)
            if (dist < nn_dist_threshold):
                if self.check_collision(q_nnode, qrand_node) is False:
                    # qrand_node.parent = q_nnode
                    # qrand_node.cost = q_nnode.cost + dist
                    # self.vertices.append(qrand_node)
                    neighbors = self.get_neighbors(qrand_node, neighbor_size)
                    self.rewire(qrand_node, neighbors)
            
        # set found flag
        # Finding nearest node to goal
        for node in self.vertices:
            if self.dis(node, self.goal) < goal_region:
                self.goal.parent = node
                self.goal.cost = node.cost + self.dis(node, self.goal)
                self.found = True

        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("----------------------%s------------------------" % "RRT_Star")
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
            print("----------------------------------------------------------------")
        else:
            print("No path found")

        # Draw result
        self.draw_map()

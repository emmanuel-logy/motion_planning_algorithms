# Standard Algorithm Implementation
# Sampling-based Algorithms PRM

import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
from scipy import spatial
from networkx.classes.function import neighbors
import math

# Class for PRM
class PRM:
    # Constructor
    def __init__(self, map_array):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.samples = []                     # list of sampled points
        self.graph = nx.Graph()               # constructed graph
        self.path = []                        # list of nodes of the found path
        self.dist_threshold = 0
        self.search_radius = 0

    def check_collision(self, p1, p2):
        '''Check if the path between two points collide with obstacles
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            True if there are obstacles between two points
        '''
        ### YOUR CODE HERE ###
        # step_size = 0.5        
        # num_of_steps = int(self.dis(p1, p2) / step_size)
        # x1,y1 = p1
        # x2,y2 = p2
        #
        # if (x2-x1) != 0:     # when slope is not inf
        #     m = (y2-y1) / (x2-x1)
        #
        #     for delta_x in range(num_of_steps):
        #         x = x1+delta_x
        #         y = math.floor( m*(x) )
        #         if abs(y)<self.size_row and abs(x)<self.size_col and self.map_array[x][y] == 0:
        #             return True
        #
        # else:   # when slope is inf
        #     x = x1 # or x2 since both are same
        #     y = y1
        #     for delta_y in range(num_of_steps):
        #         if (y2>y1): # walk upwards
        #             y = math.floor( y1+delta_y )
        #         else:       # walk downwards
        #             y = math.floor( y1-delta_y )
        #
        #         if abs(y)<self.size_row and abs(x)<self.size_col and self.map_array[x][y] == 0:
        #             return True
        #
        # return False
        
        # My above implementation wasn't working in certain conditions, so used this from stackoverflow 
        pts = zip(np.linspace(p1[0], p2[0]).astype(int), np.linspace(p1[1], p2[1]).astype(int))
        
        for p in pts:
            if self.map_array[p[0], p[1]] == 0:
                return True
        return False


    def dis(self, point1, point2):
        '''Calculate the euclidean distance between two points
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            euclidean distance between two points
        '''
        ### YOUR CODE HERE ###
        return spatial.distance.euclidean(point1, point2)


    def uniform_sample(self, n_pts):
        '''Use uniform sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valid points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        # Calculating number of rows and cols based on total points to be used
        samples_rowwise = math.floor(math.sqrt(n_pts))
        # end with size-1 since samples on boundaries are useless
        index = np.linspace(0, self.size_row-1, samples_rowwise).astype(int) 
        for r in index:
            for c in index:
                if self.map_array[r][c] == 1:
                    self.samples.append((r,c))
        
        # min dis between diagonal neighbors using pythagoras thm...plus 5 allow little extra boundary of radius
        self.dist_threshold = math.sqrt( (index[1]-index[0])**2 + (index[1]-index[0])**2 ) + 2
        self.search_radius = self.dist_threshold + 5
        # print (self.samples)

    
    def random_sample(self, n_pts):
        '''Use random sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        rng = np.random.default_rng()
        pos = rng.integers(0, self.size_row-1, (n_pts,2)).astype(int)
        for pt in pos:
            if self.map_array[pt[0]][[pt[1]]] == 1:
                self.samples.append(pt)

        self.dist_threshold = 20
        self.search_radius = self.dist_threshold + 5
        # print (self.samples)
        

    # Ref: https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/PRM/prmsampling_02.pdf    
    def gaussian_sample(self, n_pts):
        '''Use gaussian sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        rng = np.random.default_rng()
        self.dist_threshold = 25    # this is for nearest neighbor search during building the map
         # this depends on the terrain... if the map has obstacles only in one corner then all samples will be only in that corner
         # and when start and goal are far and threshold is limited, then start and goal cant connect to this map!!
        self.search_radius = 65       # this is for connecting goal and start to the graph
        
        itr = 0
        while itr <= n_pts:
            itr = itr+1
            c1 = rng.random((1,2))                # c1 will be in interval [0,1)
            c1 = (c1*self.size_row).astype(int) # scale c1 according to size of image
        
            mean = c1
            sd = 2      # lesser it is, closer the sample will be towards the obstacle
            pos = rng.normal(mean, sd, (30,2)).astype(int)   # 20*2 matrix.. i.e for lesser computation time, just look at 30 samples around c1
            for c2 in pos:
                if c2[0] >= self.size_row or c2[1] >= self.size_col:
                    continue
                if self.map_array[c1[0][0]][c1[0][1]] == 1 and self.map_array[c2[0]][[c2[1]]] == 0: # one of them should be in collision
                    self.samples.append(c1[0]) 
                    break
                elif self.map_array[c1[0][0]][c1[0][1]] == 0 and self.map_array[c2[0]][[c2[1]]] == 1: # one of them should be in collision
                    self.samples.append(c2)
                    break
        # print (self.samples)
                

    # Ref: https://ieeexplore.ieee.org/abstract/document/1242285/similar#similar
    def bridge_sample(self, n_pts):
        '''Use bridge sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        rng = np.random.default_rng()
        self.dist_threshold = 25    # this is for nearest neighbor search during building the map
         # this depends on the terrain... if the map has obstacles only in one corner then all samples will be only in that corner
         # and when start and goal are far and threshold is limited, then start and goal cant connect to this map!!
        self.search_radius = 65       # this is for connecting goal and start to the graph
        
        itr = 0
        while itr <= n_pts:
            itr = itr+1
            c1 = rng.random((1,2))                # c1 will be in interval [0,1)
            c1 = (c1*self.size_row).astype(int) # scale c1 according to size of image

            if self.map_array[c1[0][0]][c1[0][1]] == 0:     # Go ahead only if c1 is in collision
                mean = c1
                sd = 2      # lesser it is, closer the sample will be towards the obstacle.. lesser it is, more narrower passage can be caught
                pos = rng.normal(mean, sd, (20,2)).astype(int)   # 20*2 matrix.. i.e for lesser computation time, just look at 30 samples around c1
                for c2 in pos:
                    if c2[0] >= self.size_row or c2[1] >= self.size_col:
                        continue
                    if self.map_array[c2[0]][[c2[1]]] == 0:     # Go ahead only if second pt is also in collision
                        dist = self.dis(c1[0],c2)
                        # Get midpoint
                        mid_pt = dist/2
                        c3_x=0
                        if c1[0][0] < c2[0]:
                            c3_x = math.floor(c1[0][0] + mid_pt)
                        else:
                            c3_x = math.floor(c1[0][0] - mid_pt)
                        
                        c3_y=0
                        if c1[0][1] < c2[1]:
                            c3_y = math.floor(c1[0][1] + mid_pt)
                        else:
                            c3_y = math.floor(c1[0][1] - mid_pt)
                        c3 = (c3_x, c3_y)
                        
                        if self.map_array[c3[0]][c3[1]] == 1:   # if mid-pt is coliision free, then append sample
                            self.samples.append(c3) 
                            break             
        # print (self.samples)


    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots()
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw graph
        # get node position (swap coordinates)
        node_pos = np.array(self.samples)[:, [1, 0]]
        pos = dict( zip( range( len(self.samples) ), node_pos) )
        pos['start'] = (self.samples[-2][1], self.samples[-2][0])
        pos['goal'] = (self.samples[-1][1], self.samples[-1][0])
        
        # draw constructed graph
        nx.draw(self.graph, pos, node_size=3, node_color='y', edge_color='y' ,ax=ax)

        # If found a path
        if self.path:
            # add temporary start and goal edge to the path
            final_path_edge = list(zip(self.path[:-1], self.path[1:]))
            nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=self.path, node_size=8, node_color='b')
            nx.draw_networkx_edges(self.graph, pos=pos, edgelist=final_path_edge, width=2, edge_color='b')

        # draw start and goal
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['start'], node_size=12,  node_color='g')
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['goal'], node_size=12,  node_color='r')

        # show image
        plt.axis('on')
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.show()


    def sample(self, n_pts=1000, sampling_method="uniform"):
        '''Construct a graph for PRM
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            sampling_method - name of the chosen sampling method

        Sample points, connect, and add nodes and edges to self.graph
        '''
        # Initialize before sampling
        self.samples = []
        self.graph.clear()
        self.path = []

        # Sample methods
        if sampling_method == "uniform":
            self.uniform_sample(n_pts)
        elif sampling_method == "random":
            self.random_sample(n_pts)
        elif sampling_method == "gaussian":
            self.gaussian_sample(n_pts)
        elif sampling_method == "bridge":
            self.bridge_sample(n_pts)

        ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # Store them as
        # pairs = [(p_id0, p_id1, weight_01), (p_id0, p_id2, weight_02), 
        #          (p_id1, p_id2, weight_12) ...]
        kdtree = spatial.KDTree(self.samples)
        pairs = []
                        
        i = 0 # to avoid connecting a node to itself
        for pt in self.samples:
            neighbors = kdtree.query_ball_point(pt, self.dist_threshold)
            for index in neighbors:
                if index != i:
                    a = pt
                    b = self.samples[index]
                    # print((i,index,spatial.distance.euclidean(a,b)))
                    if self.check_collision(a,b) is False:
                        pairs.append((i, index, self.dis(a,b)))  
            i = i+1


        # Use sampled points and pairs of points to build a graph.
        # To add nodes to the graph, use
        # self.graph.add_nodes_from([p_id0, p_id1, p_id2 ...])
        # To add weighted edges to the graph, use
        # self.graph.add_weighted_edges_from([(p_id0, p_id1, weight_01), 
        #                                     (p_id0, p_id2, weight_02), 
        #                                     (p_id1, p_id2, weight_12) ...])
        # 'p_id' here is an integer, representing the order of 
        # current point in self.samples
        # For example, for self.samples = [(1, 2), (3, 4), (5, 6)],
        # p_id for (1, 2) is 0 and p_id for (3, 4) is 1.
        self.graph.add_nodes_from(range(len(self.samples)))
        self.graph.add_weighted_edges_from(pairs)

        # Print constructed graph information
        n_nodes = self.graph.number_of_nodes()
        n_edges = self.graph.number_of_edges()
        print("----------------------PRM_%s------------------------" % sampling_method)
        print("The constructed graph has %d nodes and %d edges" %(n_nodes, n_edges))


    def search(self, start, goal):
        '''Search for a path in graph given start and goal location
        arguments:
            start - start point coordinate [row, col]
            goal - goal point coordinate [row, col]

        Temporary add start and goal node, edges of them and their nearest neighbors
        to graph for self.graph to search for a path.
        '''
        # Clear previous path
        self.path = []

        # Temporarily add start and goal to the graph
        self.samples.append(start)
        self.samples.append(goal)
        # start and goal id will be 'start' and 'goal' instead of some integer
        self.graph.add_nodes_from(['start', 'goal'])

        ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # You could store them as
        # start_pairs = [(start_id, p_id0, weight_s0), (start_id, p_id1, weight_s1), 
        #                (start_id, p_id2, weight_s2) ...]
        kdtree = spatial.KDTree(self.samples)
        
        start_pairs = []
        neighbors = kdtree.query_ball_point(start, self.search_radius)
        for index in neighbors:
            if index != 0:
                a = start
                b = self.samples[index]
                if self.check_collision(a,b) is False:
                    start_pairs.append(('start', index, self.dis(a,b)))
                   
                
        goal_pairs = []
        neighbors = kdtree.query_ball_point(goal, self.search_radius)
        for index in neighbors:
            if index != 0:
                a = goal
                b = self.samples[index]
                if self.check_collision(a,b) is False:
                    goal_pairs.append(('goal', index, self.dis(a,b)))
                   
       

        # Add the edge to graph
        self.graph.add_weighted_edges_from(start_pairs)
        self.graph.add_weighted_edges_from(goal_pairs)
        
        # Seach using Dijkstra
        try:
            self.path = nx.algorithms.shortest_paths.weighted.dijkstra_path(self.graph, 'start', 'goal')
            path_length = nx.algorithms.shortest_paths.weighted.dijkstra_path_length(self.graph, 'start', 'goal')
            print("The path length is %.2f" %path_length)
            print("----------------------------------------------------------------")
        except nx.exception.NetworkXNoPath:
            print("No path found")
        
        # Draw result
        self.draw_map()

        # Remove start and goal node and their edges
        self.samples.pop(-1)
        self.samples.pop(-1)
        self.graph.remove_nodes_from(['start', 'goal'])
        self.graph.remove_edges_from(start_pairs)
        self.graph.remove_edges_from(goal_pairs)
        
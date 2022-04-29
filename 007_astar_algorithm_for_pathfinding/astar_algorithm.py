# A* algorithm implementation
# by Prof. Dr. Juergen Brauer, www.juergenbrauer.org
#
# Special thanks go to "Shakey, the Robot" ands its developer team
# for this great algorithm!

import numpy
import cell_types

class astar:

    def __init__(self, map, start, goal):

        print("Creating A* algorithm object")

        self.map   = map
        self.start = start
        self.goal  = goal

        # initialize node infos
        # needed for the algorithm
        # for each cell we store:
        #  - its g-costs
        #  - its h-costs
        #  - its f-costs (yes, its redundant since f=g+h)
        #  - from which node we reached this node
        self.node_infos = {}
        self.grid_height = self.map.shape[0]
        self.grid_width  = self.map.shape[1]
        for cell_y in range(0,self.grid_height):
            for cell_x in range(0, self.grid_width):
                
                # is this the start node?
                if (cell_x,cell_y) == self.start:
                    # the g-costs for the start node is zero,
                    # since there are no costs to get from the start
                    # to the start ;-)
                    g_costs = 0
                else:
                    # for all other nodes the g-costs start with inf(inity),
                    # since we do not know how to get there at all
                    g_costs = numpy.inf
                
                # the-h costs for a node are computed using a heuristic function                
                h_costs = self.heuristic( (cell_x,cell_y), self.goal )

                # f=g+h
                f_costs = g_costs + h_costs

                # node_infos is a dictionary that maps cells to a small
                # dictionary that contains all the information for this cell
                self.node_infos[ (cell_x,cell_y) ] = \
                    {"g": g_costs,
                     "h": h_costs,
                     "f": f_costs,
                     "came_from" : None,                     
                     }

        # put the start node into the open list of nodes to expand
        self.openlist = [self.start]

        # the following list of offset coordinates (dx,dy)
        # defines what the neighbors of a node are
        self.neighbors_offsets = [ (-1,-1), (0,-1), (+1,-1),
                                   (-1, 0),         (+1, 0),
                                   (-1,+1), (0,+1), (+1,+1)
                                 ]


    def heuristic(self, nodeA, nodeB):
        dx = nodeA[0] - nodeB[0]
        dy = nodeA[1] - nodeB[1]
        dist = int(numpy.sqrt(dx**2 + dy**2) * 10)
        return dist    
                


    def get_neighbors(self, node):
        
        neighbors = []
                
        for offset in self.neighbors_offsets:

            # compute grid coordinates of neighbor
            other_x = node[0] + offset[0]
            other_y = node[1] + offset[1]

            # is this a valid neighbor?

            # neighbor coordinates valid?
            if other_x < 0 or other_x>=self.grid_width or \
               other_y < 0 or other_y>=self.grid_height:
               # coordinates of neighbor are invalid (out of grid range)!
               continue

            # neighbor walkable?                
            celltype = self.map[ other_y, other_x ]
            if celltype == cell_types.celltype_wall:
                # neighbor cell is a wall, so it is not walkable!
                continue

            neighbors.append( (other_x, other_y) )

        return neighbors
        


    def single_step(self):

        print("single A* step")

        # 1. are there still nodes to expand?
        if len(self.openlist) == 0:
            print("Sorry! No more nodes to expand!")
            return self.node_infos
        

        # 2. search for the node in the open list
        #    that has the smallest f-costs
        min_f_costs_found_so_far = None
        node_with_smallest_f_costs = None
        for node in self.openlist:

            # just take the first node, if we have no
            # node to expand so far
            if node_with_smallest_f_costs == None:
                node_with_smallest_f_costs = node
                min_f_costs_found_so_far = self.node_infos[node]["f"]
                continue

            # what are the f-costs for this node?
            f_costs = self.node_infos[node]["f"]

            # did we find a better node?
            if f_costs < min_f_costs_found_so_far:
                min_f_costs_found_so_far = f_costs
                node_with_smallest_f_costs = node
        
        print("Best node to expand is: ", node_with_smallest_f_costs )        


        # 3. now that we have decided for a node to expand,
        #    expand it!
        node_to_expand = node_with_smallest_f_costs
        neighbors = self.get_neighbors( node_to_expand )
        for neighbor in neighbors:
            print(neighbor)
             

               

        

        return self.node_infos


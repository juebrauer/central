# A* algorithm implementation
# by Prof. Dr. Juergen Brauer, www.juergenbrauer.org
#
# Special thanks go to "Shakey, the Robot" ands its developer team
# for this great algorithm!

import numpy
import cell_types
import astar_node_types

class astar:

    def __init__(self, map, start, goal):

        print("Creating A* algorithm object.")
        print("Costs are: f=g+h")
        print("           total estimated costs = costs so far to reach the cell + heuristic costs")

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
                     "node_type" : astar_node_types.nodetype_unknown                     
                     }

        # put the start node into the open set
        # this is the set of nodes to expand
        self.openset = { self.start }
        self.node_infos[ self.start ]["node_type"] = astar_node_types.nodetype_open

        # we also maintain a list of nodes that
        # we have already expanded
        self.closedset = set()

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



    def get_node_with_smallest_f_costs(self):
        
        min_f_costs_found_so_far = None
        node_with_smallest_f_costs = None

        # for all open nodes:
        # find the one with the smallest f costs
        for node in self.openset:

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

        # return the node that is in the open set
        # and has the smallest f-costs
        return node_with_smallest_f_costs
        
    

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


    def reconstruct_path(self):

        current = self.goal
        path = [current]

        while current != self.start:            
            # how did we get to node "current"?
            current = self.node_infos[current]["came_from"]
            path.insert(0, current)

        print("Path: ", end="")
        for node in path:
            print( node, end=" ")
            self.node_infos[node]["node_type"] = astar_node_types.nodetype_path
        print("")



    def single_step(self):

        print("single A* step")

        # 1. are there still nodes to expand?
        if len(self.openset) == 0:
            print("Sorry! No more nodes to expand!")
            return self.node_infos
        

        # 2. search for the node in the open set
        #    that has the smallest f-costs
        node_to_expand = self.get_node_with_smallest_f_costs()
        estimated_costs = self.node_infos[node_to_expand]["f"]
        print( f"Best node to explore/expand is {node_to_expand}  with estimated path costs of {estimated_costs}")


        # 3. is the considered node_to_expand the goal node?
        if node_to_expand == self.goal:
            print("FOUND A PATH TO THE GOAL!")
            self.reconstruct_path()
            return self.node_infos 
            

        # 4. remove the node from the open set
        self.openset.remove( node_to_expand )


        # 5. put the node in to list of nodes
        #    that we have already expanded
        self.closedset.add( node_to_expand )
        self.node_infos[ node_to_expand ]["node_type"] = \
            astar_node_types.nodetype_closed


        # 6. now "expand" the node!
        #    i.e. compute tentative g-costs for these nodes
        #         and update f-costs
        neighbors = self.get_neighbors( node_to_expand )
        for neighbor in neighbors:

            #print(neighbor)

            # 6.1
            # if this neighbor has been expanded already
            # before, i.e. is already in the closed set,
            # do nothing
            if neighbor in self.closedset:
                continue
            
            # 6.2
            # what are the costs to go from the start node
            # to the current node to be expanded?
            gcosts_node_to_expand = self.node_infos[ node_to_expand ]["g"]


            # 6.3
            # compute a new estimate for the g-costs of this node 'neighbor'
            # why are theses costs 'tentative'?
            # well, perhaps we find a short path to 'neighbor' from the
            # start node in the future!
            tentative_gcosts_for_neighbor = gcosts_node_to_expand + self.heuristic(node_to_expand, neighbor)


            # 6.4
            # how long was the best path from the start node
            # to the 'neighbor' node that we knew so far?
            old_known_gcosts_for_neighbor = self.node_infos[ neighbor ]["g"]


            # 6.5
            # so: did we find a better path to node 'neighbor'?
            if tentative_gcosts_for_neighbor < old_known_gcosts_for_neighbor:

                # yes! we did!

                # so a better way to node 'neighbor' is to go there
                # from node 'node_to_expand'
                self.node_infos[ neighbor ][ "came_from" ] = node_to_expand

                # update g-costs for node 'neighbor'
                self.node_infos[ neighbor ][ "g" ] = tentative_gcosts_for_neighbor

                # if the g-costs are smaller, the f-costs (overall costs to go from 
                # start to goal) are also, since f:=g+h
                self.node_infos[ neighbor ][ "f" ] = tentative_gcosts_for_neighbor + \
                                                     self.node_infos[ neighbor ][ "h" ]
                
                # add this node 'neighbor' to the open set
                # since we need to expand this node
                # in the future as well if it promises a
                # short path from the start to the goal node                
                self.openset.add( neighbor )
                self.node_infos[ neighbor ]["node_type"] = \
                    astar_node_types.nodetype_open

        return self.node_infos


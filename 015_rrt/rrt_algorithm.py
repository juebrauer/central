import numpy
import nodes_and_trees
from PySide6 import QtGui
import params


class rrt:

    def __init__(self, map, start, goal):
        self.map = map

        # get a single pixel value
        #pixel_value = map.pixel(0,0)
        #c = QtGui.QColor(pixel_value)  # color object
        #print( f"pixel value={pixel_value} and RGB={c.getRgb()}" )
        #print( c.red() )


        self.map_width = map.width()
        self.map_height = map.height()

        self.start = start
        self.goal = goal

        self.tree = nodes_and_trees.tree()
        
        n = nodes_and_trees.node(start[0], start[1], None)
        self.tree.nodes.append( n )

        self.step = 0

        self.last_rnd_point = None

        self.path_from_start_to_goal = None

        

    def get_random_location(self):
        rndx = numpy.random.randint(low=0, high=self.map_width-1)
        rndy = numpy.random.randint(low=0, high=self.map_height-1)
        return rndx,rndy


    def distance(self, p1, p2):
        return numpy.linalg.norm( numpy.array(p1) - numpy.array(p2) )


    def get_nearest_node_and_distance_to_location(self, x,y):

        nearest_node = None
        min_dist = None
        for node in self.tree.nodes:

            if nearest_node == None:
                nearest_node = node
                min_dist = self.distance( (x,y), (node.x,node.y) )
                continue

            dist = self.distance( (x,y), (node.x,node.y) )

            if dist < min_dist:
                nearest_node = node
                min_dist = dist
        
        return nearest_node, min_dist


    def compute_new_location_candidate(self, node, x,y):

        # 1. compute vector from node to (x,y)
        vecx = x - node.x
        vecy = y - node.y

        # 2. compute unit direction vector (dirx,diry)
        len_vec = numpy.linalg.norm((vecx,vecy))
        dirx = vecx/len_vec
        diry = vecy/len_vec

        # 3. go from node into direction (dirx,diry)
        #    a distance of incr_dist
        finalx = int(node.x + dirx*params.ALGO_INCR_DIST)
        finaly = int(node.y + diry*params.ALGO_INCR_DIST)
        
        return finalx, finaly


    def is_walkable(self, p1, p2):

         # 1. compute vector pointing from p1 to p2
        vecx = p2[0] - p1[0]
        vecy = p2[1] - p1[1]

        # 2. go step by step from p1 to p2 and check
        #    whether intermediate points are walkable ("free")
        for r in numpy.linspace(0.0, 1.0, 100):
            ix = int(p1[0] + r * vecx)
            iy = int(p1[1] + r * vecy)

            pixel_value = self.map.pixel(ix,iy)
            c = QtGui.QColor(pixel_value)
            
            if (c.red(), c.green(), c.blue()) != params.ALGO_MAP_COLOR_WALKABLE:

                #print("Found a not walkable intermediate pixel!")

                # we found a not walkable intermediate pixel!
                return False

        # All intermediate pixels from p1 to p2
        # have a color that is walkable (e.g., white)!
        # So the path from p1 to p2 is walkable.
        return True



    def reconstruct_path(self, n):

        path = []
        loc = None

        while loc != self.start:
            loc = (n.x, n.y)
            #print( f"{loc}", end=" " )

            path.insert(0,loc)           

            # go to parent node
            n = n.parent

        # last node is start node,
        # insert start node as well
        path.insert(0,loc)

        return path

            

    def run_single_step(self):
        #print("RRT single step")

        self.step += 1

        # 1. get random location
        rndx, rndy = self.get_random_location()
        self.last_rnd_point = (rndx, rndy)
                        
        # 2. determine node that is next to this location
        nearest_node, dist = \
            self.get_nearest_node_and_distance_to_location(rndx,rndy)

        # catch a special case:
        # the random location was exactly
        # placed on an existing node in the tree!
        # then the direction vector is the null vector
        # and "going into the direction" of the random
        # location from the nearest node in the tree does not work!
        if dist == 0:
            return self.tree

        # 3. compute new location candidate
        candx, candy = \
            self.compute_new_location_candidate(nearest_node,
                                                rndx,rndy)

        # 4. is the candidate location still in the map?
        if candx<0 or candx>=self.map_width or \
           candy<0 or candy>=self.map_height:
           # No!
           return self.tree
        
        # 5. check whether the path from nearest_node to (candx,candy)
        #    is walkable
        if self.is_walkable( (nearest_node.x, nearest_node.y),
                             (candx, candy)):
            n = nodes_and_trees.node(candx, candy, nearest_node)
            self.tree.nodes.append( n )

            # is the new node in the termination neighborhood
            # of the goal node?
            p1 = (n.x, n.y)
            p2 = self.goal
            dist = self.distance(p1,p2)
            if dist <= params.ALGO_TERMINATION_RADIUS:
                print("New node is in the termination radius to the goal node!")

                self.path_from_start_to_goal = self.reconstruct_path(n)
        
        
        return self.tree





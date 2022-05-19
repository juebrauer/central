import numpy
import nodes_and_trees
from PySide6 import QtGui
import params


class rrt:

    def __init__(self, map, start, goal, incr_dist=10):
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

        self.incr_dist = incr_dist


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


    def compute_new_location_candidate(self, node, x,y, incr_dist):

        # 1. compute vector from node to (x,y)
        vecx = node.x - x
        vecy = node.y - y

        # 2. compute unit direction vector
        len_vec = numpy.linalg.norm((vecx,vecy))
        dirx = vecx/len_vec
        diry = vecy/len_vec

        # 3. go from node into direction (dirx,diry)
        #    a distance of incr_dist
        finalx = node.x + dirx*incr_dist
        finaly = node.y + diry*incr_dist

        return finalx, finaly


    def is_walkable(self, p1, p2):

         # 1. compute vector pointing from p1 to p2
        vecx = p2[0] - p1[0]
        vecy = p2[1] - p1[1]

        # 2. compute unit direction vector
        len_vec = numpy.linalg.norm((vecx,vecy))
        dirx = vecx/len_vec
        diry = vecy/len_vec

        # 3. go step by step from p1 to p2 and check
        #    whether intermediate points are walkable ("free")
        for r in numpy.linspace(0.0, 1.0, 100):
            ix = int(p1[0] + r * dirx)
            iy = int(p1[1] + r * diry)

            pixel_value = self.map.pixel(ix,iy)
            c = QtGui.QColor(pixel_value)
            
            if (c.red(), c.green(), c.blue()) != params.COLOR_WALKABLE:

                # we found a not walkable intermediate pixel!
                return False

        # All intermediate pixels from p1 to p2
        # have a color that is walkable (e.g., white)!
        # So the path from p1 to p2 is walkable.
        return True

            

    def run_single_step(self):
        print("RRT single step")

        self.step += 1

        # 1. get random location
        rndx, rndy = self.get_random_location()
                        
        # 2. determine node that is next to this location
        nearest_node, dist = \
            self.get_nearest_node_and_distance_to_location(rndx,rndy)

        # 3. compute new location candidate
        candx, candy = \
            self.compute_new_location_candidate(nearest_node,
                                                rndx,rndy,
                                                self.incr_dist)
        
        # 4. check whether the path from nearest_node to (candx,candy)
        #    is walkable
        if self.is_walkable( (nearest_node.x, nearest_node.y),
                             (candx, candy)):
            n = nodes_and_trees.node(candx,candy, nearest_node)
            self.tree.nodes.append( n )

        print( f"nr_nodes={len(self.tree.nodes)}" )

        return self.tree       





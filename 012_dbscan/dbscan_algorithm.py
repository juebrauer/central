import numpy


class dbscan:
    
    def __init__(self, epsilon, min_pts):
        self.epsilon = epsilon
        self.min_pts = min_pts


    def distance_function(self, a,b):
        """
        Computes the distance between two
        data points a and b
        """
        euclidean_distance = numpy.linalg.norm(numpy.array(a)-numpy.array(b))
        return euclidean_distance


    def range_query(self, q, data_points):
        """
        For a given point p we return
        a set of "neighbors", i.e., points,
        that are within an epsilon environment
        around that point q
        (including the point q itself)
        """

        # 1. start with an empty set
        neighbors = set()

        # 2. check for each other point whether
        #    it is within the epsilon environment
        for p in data_points:

            if self.distance_function(q,p) <= self.epsilon:
                neighbors.add( p )
        
        # 3. return the list of all neighbors
        return neighbors

    # end function range_query()





    def cluster(self, data_points):
        """
        Decides for each data point whether it is a
        core/border point or noise and
        assigns a cluster id to this data point
        """

        print( f"Clustering {len(data_points)} data points with DBSCAN ...")

        # we start with an empty dictionary
        data_points_labels = {}
        
        cluster_id = 0

        # 1. label all data points initially as undefined
        for p in data_points:
            # create an empty dictionary for this data point
            data_points_labels[p] = {}
            
            data_points_labels[p]["data_point_type"] = None
            data_points_labels[p]["cluster_id"] = 0


        # 2. now decide for each data point its type and cluster id
        for p in data_points:

            # 2.1 data point processed in inner loop before?
            if data_points_labels[p]["data_point_type"] != None:
                # we already have processed this point and
                # decided which type this data point is,
                # so continue with the next point p in data_points
                continue

            # 2.2 get a list of all neighbors of this point
            neighbors = self.range_query(p, data_points)

            # 2.3 is this point a core point, i.e.,
            #     are there at least min_pts neighbors?
            N = len(neighbors)
            if N < self.min_pts:

                # no! there are not a least min_pts neighbors!
                # our first guess is that this data point is noise
                # but note:
                # later, it could be a marked as belonging to a cluster,
                # since itself is not a core point, but it could be a border
                # point which connects to another point that is a core point
                data_points_labels[p]["data_point_type"] = "noise"

                # continue with the next data point
                continue



            # we have found a core point, i.e.,
            # p has at least min_pts neighbors
            # (it is densely connected to other data points)

            # 2.4 compute next cluster id
            cluster_id += 1

            # 2.5 label the current data point p as a core point
            #     and assign the new cluster id to this point
            data_points_labels[p]["data_point_type"] = "core"
            data_points_labels[p]["cluster_id"] = cluster_id

            # 2.6 now we grow this cluster starting with the neighbors
            # seedset = neighbours \ {p}
            seedset = neighbors.difference( {p} )

            # 2.7 for each point in the seedset
            while len(seedset) != 0:

                # get next element of seedset to process
                q = seedset.pop()

                # a previously assigned noise point
                # which is a neighbor point
                # now becomes a border point
                # congrats, dear q, to the updgrade! =)
                if data_points_labels[q]["data_point_type"] == "noise":
                    data_points_labels[q]["data_point_type"] = "border"
                    data_points_labels[q]["cluster_id"] = cluster_id

                # if q is a border point or a previously assigned core point,
                # we do not process it further and
                # continue with the next seed point q
                if data_points_labels[q]["data_point_type"] != None:
                    continue

                # q is now at least a border point,
                # perhaps it is also a core point if it
                # has enough neighbors in its epsilon-environment
                data_points_labels[q]["data_point_type"] = "border"
                data_points_labels[q]["cluster_id"] = cluster_id

                # get a list of all neighbors of q (including q itself)
                neighbors_of_q = self.range_query(q, data_points)

                # is q a core point?
                if len(neighbors_of_q) >= self.min_pts:

                    # yes! q is also a dense point, a.k.a. as core point

                    # mark q as a core point
                    data_points_labels[q]["data_point_type"] = "core"

                    # add all neighbors of q itself to the set of
                    # data points to consider to belong to the current cluster
                    seedset = seedset | neighbors_of_q  # | means set union
                                    

            # end for q in seedset

        # end for p in data_points

        print( "Clustering finished." )
        print( f"Found {cluster_id} cluster(s)." )


        # 3. return the clustering result,
        # i.e.,
        # - which data point belongs to which cluster and
        # - which type each data point is (core/border or noise)
        return data_points_labels

    # end function cluster()

import numpy as np

alpha = 0.01

class neuron:

    id = 0

    def __init__(self, input_dim):
        self.weight_vec = np.random.uniform(low=0.0, high=1.0, size=input_dim)
        self.id = neuron.id
        neuron.id += 1

    def compute_act(self, inputvec):
        self.act = np.linalg.norm( inputvec - self.weight_vec )
        
    def adapt_to_vec(self, inputvec, neighborhood_value):
        self.weight_vec += (inputvec - self.weight_vec) * alpha * neighborhood_value
    


class som:

    def __init__(self, input_dim, map_side_len):
                
        self.input_dim = input_dim
        self.map_side_len = map_side_len
        self.nr_neurons = map_side_len**2

        # start with numbering the neurons from 0
        neuron.id = 0

        # create all neurons now
        self.list_neurons = []
        for neuron_nr in range(self.nr_neurons):
            n = neuron(input_dim)
            self.list_neurons.append( n )

        print( f"I have generated {len(self.list_neurons)} neurons." )
        
        
    def get_neighbors(self, BMU):
        
        """
        Example IDs for a SOM map with 7x7=49 neurons:
        
        array([[ 0,  1,  2,  3,  4,  5,  6],
               [ 7,  8,  9, 10, 11, 12, 13],
               [14, 15, 16, 17, 18, 19, 20],
               [21, 22, 23, 24, 25, 26, 27],
               [28, 29, 30, 31, 32, 33, 34],
               [35, 36, 37, 38, 39, 40, 41],
               [42, 43, 44, 45, 46, 47, 48]])
        """
        
        I = BMU.id
        S = self.map_side_len
        N = self.nr_neurons
        
        # Divide-and-Conquer
        
        # case 1: corner neuron
        
        # top left corner
        if I==0:
            return [1,S]
        
        # top right corner
        if I==S-1:
            return [S-2, 2*S-1]
        
        # bottom left corner
        if I==N-S:
            return [N-S+1, N-2*S]
        
        # bottom right corner
        if I==N-1:
            return [N-2, N-1-S]
        
        
        # case 2: border neuron
        
        y = I//S
        x = I%S
        
        # top border
        if y==0:
            return [I-1, I+1, I+S]
        
        # bottom border
        if y==S-1:
            return [I-1, I+1, I-S]
        
        # left border
        if x==0:
            return [I+1, I-S, I+S]
        
        # right border
        if x==S-1:
            return [I-1, I-S, I+S]
        
        
        # case 3: normal cell (inner neuron)
        
        return [I-1, I+1, I-S, I+S]
        
        
    def get_neuron_from_id(self, neuron_id):
        
        return self.list_neurons[neuron_id]
        


    def adapt(self, inputvec):

        # 1. determine BMU (best matching unit)
        BMU = None
        for n in self.list_neurons:
            n.compute_act( inputvec )

            if BMU==None or n.act < BMU.act:
                BMU = n
        
        # 2. get list of all neighbors for this BMU
        list_of_neighbor_ids = self.get_neighbors( BMU )        
        
        # 3. adapt BMU and neighbors to input vector
        BMU.adapt_to_vec(inputvec, 1.0)
        
        for neuron_id in list_of_neighbor_ids:
            n = self.get_neuron_from_id( neuron_id )
            n.adapt_to_vec(inputvec, 0.5)
        



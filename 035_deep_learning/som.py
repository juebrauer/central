import numpy as np

class neuron:

    id = 0

    def __init__(self, input_dim):
        self.weight_vec = np.random.uniform(low=0.0, high=1.0, size=input_dim)
        self.id = neuron.id
        neuron.id += 1

    def compute_act(self, inputvec):
        self.act = np.linalg.norm( inputvec - self.weight_vec )
    


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

        print( f"I have generated {len(self.list_neurons)} many neurons." )


    def adapt(self, inputvec):

        # 1. determine BMU (best matching unit)
        BMU = None
        for n in self.list_neurons:
            n.compute_act( inputvec )

            if BMU==None or n.act > BMU.act:
                BMU = n

        print( f"id of BMU={BMU.id}" )
        



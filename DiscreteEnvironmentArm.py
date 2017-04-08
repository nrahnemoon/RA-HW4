import numpy

class DiscreteEnvironmentArm(object):

    def __init__(self, resolution, lower_limits, upper_limits):

        # Store the resolution
        self.resolution = resolution

        # Store the bounds
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits

        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension*[0]
        for idx in range(self.dimension):
            self.num_cells[idx] = numpy.ceil((upper_limits[idx] - lower_limits[idx])/resolution)


    def ConfigurationToNodeId(self, config):
        
        # TODO:
        # This function maps a node configuration in full configuration
        # space to a node in discrete space
        #
        coord = self.ConfigurationToGridCoord(config)
        node_id = self.GridCoordToNodeId(coord)
        return node_id

    def NodeIdToConfiguration(self, nid):
        
        # TODO:
        # This function maps a node in discrete space to a configuraiton
        # in the full configuration space
        #
        coord = self.NodeIdToGridCoord(nid)
        if coord==-1:
            return -1
        config = self.GridCoordToConfiguration(coord)
        return config
        
    def ConfigurationToGridCoord(self, config):
        
        # TODO:
        # This function maps a configuration in the full configuration space
        # to a grid coordinate in discrete space
        #
        coord = [0] * self.dimension
        for idx in range(self.dimension):
            coord[idx] = int(numpy.floor((config[idx]-self.lower_limits[idx]) / self.resolution))
        return coord

    def GridCoordToConfiguration(self, coord):
        
        # TODO:
        # This function smaps a grid coordinate in discrete space
        # to a configuration in the full configuration space
        #
        config = [0] * self.dimension
        for idx in range(self.dimension):
            config[idx] = self.lower_limits[idx] + (self.resolution * coord[idx]) + (self.resolution/2)
        return config

    def GridCoordToNodeId(self,coord):
        
        # TODO:
        # This function maps a grid coordinate to the associated
        # node id 

        node_id = 0
        multiplier = 1
        for idx in range(self.dimension):
            if coord[idx]<0:
                return -1
            node_id = node_id + coord[idx] * multiplier
            multiplier = multiplier * self.num_cells[idx]
        return int(node_id)

    def NodeIdToGridCoord(self, node_id):
        
        # TODO:
        # This function maps a node id to the associated
        # grid coordinate

        coord = [0] * self.dimension
        divisor = 1
        for idx in range(self.dimension):
            divisor = divisor * self.num_cells[idx]
        if node_id >= divisor:
            return -1 
        
        for idx in range(self.dimension):
            index = -1 * (idx + 1)
            divisor = divisor / self.num_cells[index]
            coord[index] = numpy.floor(node_id / divisor)
            node_id = node_id - (divisor * coord[index])
        return coord
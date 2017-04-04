import numpy
import scipy
import scipy.spatial
from DiscreteEnvironment import DiscreteEnvironment

class HerbEnvironment(object):
    
    def __init__(self, herb, resolution):
        
        self.robot = herb.robot
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)
        self.env1 = self.robot.GetEnv()
        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        upper_coord = [x - 1 for x in self.discrete_env.num_cells]
        upper_config = self.discrete_env.GridCoordToConfiguration(upper_coord)
        for idx in range(len(upper_config)):
            self.discrete_env.num_cells[idx] -= 1

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 0.7], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)
        
        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
    
    def GetSuccessors(self, node_id):

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes

        successors = []
        node_grid = self.discrete_env.NodeIdToGridCoord(node_id)
        for i in xrange(0,self.discrete_env.dimension):
            if node_grid[i]+1<=self.discrete_env.num_cells[i]:
                node_grid_temp = list(node_grid)
                node_grid_temp[i]+=1;
                temp_config = self.discrete_env.GridCoordToConfiguration(node_grid_temp)
                with self.env1:  
                    self.robot.SetDOFValues(temp_config, self.robot.GetActiveDOFIndices())
                testcollision = self.env1.CheckCollision(self.robot)
                if self.env1.CheckCollision(self.robot)==False:
                    node_ID_temp = self.discrete_env.GridCoordToNodeId(node_grid_temp)
                    successors.append(node_ID_temp)


            if node_grid[i]-1>=0:
                node_grid_temp = list(node_grid)
                node_grid_temp[i]-=1
                temp_config = self.discrete_env.GridCoordToConfiguration(node_grid_temp)
                with self.env1:  
                    self.robot.SetDOFValues(temp_config, self.robot.GetActiveDOFIndices())
                if self.env1.CheckCollision(self.robot)==False:
                    node_ID_temp = self.discrete_env.GridCoordToNodeId(node_grid_temp)
                    successors.append(node_ID_temp)

        return successors


    def ComputeDistance(self, start_id, end_id):
        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids

        dist = 0
        #start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        #end_config = self.discrete_env.NodeIdToConfiguration(end_id)
        start_grid = self.discrete_env.NodeIdToGridCoord(start_id)
        end_grid = self.discrete_env.NodeIdToGridCoord(end_id)


        #dist = numpy.linalg.norm(start_config-end_config)
        dist = scipy.spatial.distance.cityblock(start_grid,end_grid)
        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        
        cost = 0
        # start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        # end_config = self.discrete_env.NodeIdToConfiguration(end_id)
        # cost = numpy.linalg.norm(start_config-end_config)
        start_grid = self.discrete_env.NodeIdToGridCoord(start_id)
        goal_grid = self.discrete_env.NodeIdToGridCoord(goal_id)
        cost = scipy.spatial.distance.cityblock(start_grid,goal_grid)


        
        return cost

    def SetGoalParameters(self, goal_config, p = 0.2):
            self.goal_config = goal_config
            self.p = p
        

    def GenerateRandomConfiguration(self):
        num_dof = len(self.robot.GetActiveDOFIndices())
        config = [0] * num_dof

        # TODO: Generate and return a random configuration
        lower_limits, upper_limits = self.robot.GetActiveDOFLimits()

        for dim in range (num_dof):
            config[dim] = numpy.random.uniform(low=lower_limits[dim], high=upper_limits[dim]);

        return numpy.array(config)

    def Extend(self, start_config, end_config):
        
        # TODO: Implement a function which attempts to extend from 
        # a start configuration to a goal configuration
        num_dof = len(self.robot.GetActiveDOFIndices())
        steps = 25

        if(start_config == None):
            return None
        # Generate interpolations (joint by joint?)
        JointSteps = numpy.transpose(numpy.array([start_config] * steps));
        #import IPython
        #IPython.embed()
        for dim in range(num_dof):
            JointSteps[dim] = numpy.linspace(start_config[dim], end_config[dim], steps);

        for i in range(steps):
            self.robot.SetActiveDOFValues(JointSteps[:,i]);
            
            # Check collision
            for body in self.robot.GetEnv().GetBodies():
                if ((body.GetName() != self.robot.GetName() and
                    self.robot.GetEnv().CheckCollision(self.robot, body)) or
                    self.robot.CheckSelfCollision()):
                    # Check first step
                    if (i == 0): 
                        return None
                    else:
                        #JointSteps[dim] = [JointSteps[dim,i-1]] * steps
                        return None#end_config[:] = JointSteps[:,i-1]
        
        # No collision detected 
        return numpy.array(end_config)

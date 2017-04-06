import numpy, openravepy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment
import math
import scipy
import scipy.spatial

class Control(object):
    def __init__(self, omega_left, omega_right, duration):
        self.ul = omega_left
        self.ur = omega_right
        self.dt = duration

class Action(object):
    def __init__(self, control, footprint):
        self.control = control
        self.footprint = footprint

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.herb = herb
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5., -numpy.pi], [5., 5., numpy.pi]]
        self.lower_limits, self.upper_limits = self.boundary_limits
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        self.resolution = resolution
        self.ConstructActions()

    def GenerateFootprintFromControl(self, start_config, control, stepsize=0.01):

        # Extract the elements of the control
        ul = control.ul
        ur = control.ur
        dt = control.dt

        # Initialize the footprint
        config = list(start_config) # Copy
        footprint = [numpy.array([0., 0., config[2]])]
        timecount = 0.0
        while timecount < dt:
            # Generate the velocities based on the forward model
            xdot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.cos(config[2])
            ydot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.sin(config[2])
            tdot = self.herb.wheel_radius * (ul - ur) / self.herb.wheel_distance
                
            # Feed forward the velocities
            if timecount + stepsize > dt:
                stepsize = dt - timecount
            config = config + stepsize*numpy.array([xdot, ydot, tdot])
            if config[2] > numpy.pi:
                config[2] -= 2.*numpy.pi
            if config[2] < -numpy.pi:
                config[2] += 2.*numpy.pi

            footprint_config = config.copy()
            footprint_config[:2] -= start_config[:2]
            footprint.append(footprint_config)

            timecount += stepsize
            
        # Add one more config that snaps the last point in the footprint to the center of the cell
        nid = self.discrete_env.ConfigurationToNodeId(config)
        snapped_config = self.discrete_env.NodeIdToConfiguration(nid)
        snapped_config[0] -= start_config[0]
        snapped_config[1] -= start_config[1]
        footprint.append(snapped_config)

        return footprint

    def PlotActionFootprints(self, idx):

        actions = self.actions[idx]
        fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
        
        for action in actions:
            xpoints = [config[0] for config in action.footprint]
            ypoints = [config[1] for config in action.footprint]
            pl.plot(xpoints, ypoints, 'k')
                     
        pl.ion()
        pl.show()

        

    def ConstructActions(self):

        # Actions is a dictionary that maps orientation of the robot to
        #  an action set
        self.actions = dict()
              
        wc = [0., 0., 0.]
        grid_coordinate = self.discrete_env.ConfigurationToGridCoord(wc)

        # Iterate through each possible starting orientation
        for idx in range(int(self.discrete_env.num_cells[2])):
            self.actions[idx] = []
            grid_coordinate[2] = idx
            start_config = self.discrete_env.GridCoordToConfiguration(grid_coordinate)

            # TODO: Here you will construct a set of actions
            #  to be used during the planning process
            #

            self.actions[idx] = []

            speed = 1
            l_dot = 0.2 # = r
            l = self.resolution[0]
            dt = l/l_dot

            # Move Forward
            control = Control(speed, speed, dt)
            footprint = self.GenerateFootprintFromControl(start_config, control)
            self.actions[idx].append(Action(control, footprint))

            # Move Backward
            #control = Control(-speed, -speed, dt)
            #footprint = self.GenerateFootprintFromControl(start_config, control)
            #self.actions[idx].append(Action(control, footprint))         

            th_dot = 0.8 # 2 * r / L
            th = math.pi/4
            dt = th/th_dot

            # Turn Left
            control = Control(-speed, speed, dt)
            footprint = self.GenerateFootprintFromControl(start_config, control)
            self.actions[idx].append(Action(control, footprint))

             # Turn Right
            control = Control(speed, -speed, dt)
            footprint = self.GenerateFootprintFromControl(start_config, control)
            self.actions[idx].append(Action(control, footprint))


    def GetSuccessors(self, node_id):

        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids and controls that represent the neighboring
        #  nodes
        
        gridCoord = self.discrete_env.NodeIdToGridCoord(node_id)
        config = self.discrete_env.NodeIdToConfiguration(node_id)

        for action in self.actions[gridCoord[2]]:
            # Footprint was based on 0,0,th config, so need to regenerate for current config
            footprint = self.GenerateFootprintFromControl(config, action.control) 
            successors.append(Action(action.control, footprint))

        return successors

    def ComputeDistance(self, start_id, end_id):

        dist = 0

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids

        start_grid = self.discrete_env.NodeIdToGridCoord(start_id)
        end_grid = self.discrete_env.NodeIdToGridCoord(end_id)


        #dist = numpy.linalg.norm(start_config-end_config)
        dist = scipy.spatial.distance.cityblock(start_grid,end_grid)

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        
        start_grid = self.discrete_env.NodeIdToGridCoord(start_id)
        goal_grid = self.discrete_env.NodeIdToGridCoord(goal_id)
        cost = scipy.spatial.distance.cityblock(start_grid,goal_grid)

        return cost

    def IsInCollision(self, node_id):
        orig_config = self.robot.GetTransform()
        location = self.discrete_env.NodeIdToConfiguration(node_id)
        config = self.robot.GetTransform()
        config[:2,3] = location[:2]
        config[0, 0] = math.cos(location[2])
        config[0, 1] = -1 * math.sin(location[2])
        config[1, 0] = math.sin(location[2])
        config[1, 1] = math.cos(location[2])

        env = self.robot.GetEnv()
        with env:
            self.robot.SetTransform(config)   
        collision = env.CheckCollision(self.robot)
        with env:
            self.robot.SetTransform(orig_config)
        return collision

    def IsInLimits(self, node_id):
        config = self.discrete_env.NodeIdToConfiguration(node_id)
        if config == -1:
            return False
        for idx in range(len(config)):
            if config[idx] < self.lower_limits[idx] or config[idx] > self.upper_limits[idx]:
                return False
        return True

    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        pl.xlim([self.lower_limits[0], self.upper_limits[0]])
        pl.ylim([self.lower_limits[1], self.upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')
                    
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()

    def PlotAll(self,plan):
        lines=[]
        for i in range(0,numpy.size(plan,0)-1):
            start = plan[i]
            end = plan[i+1]
            temp = [start,end]
            pl.plot([start[0], end[0]],
                    [start[1], end[1]],
                    'k.-', linewidth=2.5)
        pl.draw()
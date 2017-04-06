import heapq
import time
import numpy
class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()


    def Plan(self, start_config, goal_config):
        # TODO: Here you will implement the AStar planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        start_time = time.time()

        plan = []
        openlist = []
        closedlist = []
        camefrom = dict()
        camewith = dict()
        IDandG = dict()

        startID = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        startGrid = self.planning_env.discrete_env.ConfigurationToGridCoord(start_config)

        goalID = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        goalGrid = self.planning_env.discrete_env.ConfigurationToGridCoord(goal_config)

        g_start = 0
        h_start = self.planning_env.ComputeHeuristicCost(startID,goalID)
        f_start = g_start + h_start
        heapq.heappush(openlist, (f_start,startID))

        IDandG[startID] = g_start

        ########## plot
        self.planning_env.InitializePlot(goal_config)
        ############

        while openlist:
            current = heapq.heappop(openlist)
            currentID = current[1]
            current_g = IDandG[currentID]

            if currentID == goalID:
                break

            currConfig = self.planning_env.discrete_env.NodeIdToConfiguration(currentID)
            successors = self.planning_env.GetSuccessors(currentID)

            closedlist.append(currentID)


            for i in xrange(0,len(successors)):

                successorAction = successors[i]
                numFootprints = len(successorAction.footprint)
                successorConfig = list(successorAction.footprint[numFootprints - 1])
                successorConfig[0] += currConfig[0] # Footprints only contain the relative changes in config
                successorConfig[1] += currConfig[1] # So need to add to parent for x and y to get absolute config
                successorNodeId = self.planning_env.discrete_env.ConfigurationToNodeId(successorConfig)

                if (successorNodeId ==-1 or not self.planning_env.IsInLimits(successorNodeId) or self.planning_env.IsInCollision(successorNodeId)):
                    continue
                if successorNodeId in closedlist:
                    continue
                tentative_g = current_g + self.planning_env.ComputeDistance(currentID, successorNodeId)
                if successorNodeId not in IDandG:
                    tentative_h = self.planning_env.ComputeHeuristicCost(successorNodeId, goalID)
                    tentative_f = tentative_g + tentative_h
                    heapq.heappush(openlist,(tentative_h, successorNodeId))
                    IDandG[successorNodeId] = tentative_g

                elif tentative_g >= self.planning_env.ComputeDistance(startID, successorNodeId):
                    continue

                camefrom[successorNodeId] = currentID
                camewith[successorNodeId] = successorAction

                temp_start_config = self.planning_env.discrete_env.NodeIdToConfiguration(currentID)
                temp_end_config = self.planning_env.discrete_env.NodeIdToConfiguration(successorNodeId)
                ########### plot
                self.planning_env.PlotEdge(temp_start_config, temp_end_config)
                ###########

        planID = []
        planActions = []
        #planID.append(goalID)
        search_index = goalID
        while startID not in planID:
            planID.append(search_index)
            if search_index != startID:
                planActions.append(camewith[search_index])
                search_index = camefrom[search_index]

        planID.reverse()
        planActions.reverse()

        for i in xrange(0,len(planID)):
            planConfig = self.planning_env.discrete_env.NodeIdToConfiguration(planID[i])
            plan.append(planConfig)

        end_time = time.time()
        used_time = end_time - start_time
        print "Time Used"
        print used_time
        print "Plan length: "+str(self.Plan_Length(plan))
        print "Num actions: "+str(len(planActions))
        return planActions

    def Plan_Length (self,plan): 
        dist = 0
        #self.planning_env.InitializePlot(plan[-1])
        for i in range(0,numpy.size(plan,0)-1):
            start_node = plan[i]
            end_node = plan[i+1]
            diff_vec = numpy.subtract(end_node,start_node)
            dist = dist+numpy.sqrt(numpy.dot(numpy.transpose(diff_vec),diff_vec))
        #self.planning_env.PlotAll(plan )
        return dist
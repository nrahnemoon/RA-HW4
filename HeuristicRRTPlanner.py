import numpy
from RRTTree import RRTTree
import time
import random

class HeuristicRRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize

    def Plan(self, start_config, goal_config, epsilon = .001):
        start_time = time.time()
        tree = RRTTree(self.planning_env, start_config)
        plan = []

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        
        plan.append(start_config)
        plan.append(goal_config)

        startNodeID = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goalNodeID = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        hCost = {}
        hCost[startNodeID] = 0;
        treeOptCost = self.planning_env.ComputeHeuristicCost(startNodeID, goalNodeID)
        treeMaxCost = 0

        currConfig = start_config;
        currNodeID = startNodeID;
        currID = tree.GetRootId();

        print "startConfig = [%.2f, %.2f]" %(start_config[0], start_config[1])
        print "goalConfig = [%.2f, %.2f]" %(goal_config[0], goal_config[1])
        #while (self.planning_env.Extend(currConfig, goal_config) == None):

        while (self.planning_env.ComputeDistance(currNodeID, goalNodeID) > epsilon):
            if(random.random() < .7):
                while True:
                    newCurrConfig = self.planning_env.GenerateRandomConfiguration();
                    [nearID, nearConfig] = tree.GetNearestVertex(newCurrConfig);
                    nearNodeID = self.planning_env.discrete_env.ConfigurationToNodeId(nearConfig)
                    nearCost = hCost[nearNodeID] + self.planning_env.ComputeHeuristicCost(nearNodeID, goalNodeID)
                    mQuality = (1 - ((nearCost - treeOptCost)/(treeMaxCost - treeOptCost)))
                    mQuality = max(mQuality, 0.2)
                    r = random.random()
                    print "mQuality = %.2f and r = %.2f" % (mQuality, r)
                    if (r < mQuality):
                        break;
            else:
                newCurrConfig = goal_config;
                [nearID, nearConfig] = tree.GetNearestVertex(newCurrConfig);

            print "newCurrConfig = [%.2f, %.2f]" %(newCurrConfig[0], newCurrConfig[1])
            print "nearID = %d, nearConfig = [%.2f, %.2f]" %(nearID, nearConfig[0], nearConfig[1])
            
            extension = self.planning_env.Extend(nearConfig, newCurrConfig)
            print extension

            if (extension != None):
                currConfig = extension
                currID = tree.AddVertex(currConfig);
                tree.AddEdge(nearID, currID);
                nearNodeID = self.planning_env.discrete_env.ConfigurationToNodeId(nearConfig)
                extensionNodeID = self.planning_env.discrete_env.ConfigurationToNodeId(extension)
                currNodeID = extensionNodeID
                hCost[extensionNodeID] = hCost[nearNodeID] + self.planning_env.ComputeHeuristicCost(nearNodeID, extensionNodeID)
                extensionCost = hCost[extensionNodeID] + self.planning_env.ComputeHeuristicCost(extensionNodeID, goalNodeID)
                treeMaxCost = max(extensionCost, treeMaxCost)

                #plan.append(currConfig)

                print "currID = %d, currConfig = [%.2f, %.2f]" %(currID, currConfig[0], currConfig[1])
                #self.planning_env.PlotEdge(nearConfig, currConfig);

        goalID = tree.AddVertex(goal_config);
        tree.AddEdge(currID, goalID);
        #self.planning_env.PlotEdge(currConfig, goal_config)

        currConfig = goal_config
        currID = goalID;
        while 1:
            currID = tree.edges[currID];
            currConfig = tree.vertices[currID];
            if (currID == tree.GetRootId()):
                break;
            else:
                plan.insert(1, currConfig);

        for config in plan:
            print "config = [%.2f, %.2f]" %(config[0], config[1])
        plan_length = self.Plan_Length(plan)
        print("--- %s seconds ---" % (time.time() - start_time))
        print("--- %s path length ---" % plan_length)
        print("--- %s vertices ---" % len(tree.vertices))


        return plan;


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

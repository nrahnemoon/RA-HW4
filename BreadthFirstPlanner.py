from RRTTree import RRTTree
from Queue import Queue
import time
import numpy

class BreadthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        
    def Plan(self, start_config, goal_config):
        self.planning_env.InitializePlot( goal_config)
        start_time = time.time()
        plan = []

        # TODO: Here you will implement the breadth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        
        plan.append(start_config)
        plan.append(goal_config)

        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

        seen = [ start_id ]
        
        queue = Queue()
        queue.put(start_id)

        tree = RRTTree(self.planning_env, start_config)
        nodeIdToTreeIdDict = {}
        nodeIdToTreeIdDict[start_id] = 0

        while not queue.empty():

            curr_id = queue.get()
            curr_tree_id = nodeIdToTreeIdDict[curr_id]

            for node_id in self.planning_env.GetSuccessors(curr_id):
                
                if not node_id in seen:

                    seen.append(node_id)
                    node_tree_id = tree.AddVertex(self.planning_env.discrete_env.NodeIdToConfiguration(node_id))
                    nodeIdToTreeIdDict[node_id] = node_tree_id
                    tree.AddEdge(curr_tree_id, node_tree_id)
                    #self.planning_env.PlotEdge(self.planning_env.discrete_env.NodeIdToConfiguration(curr_id),self.planning_env.discrete_env.NodeIdToConfiguration(node_id) );
                    queue.put(node_id)

                    if node_id == goal_id:
                        while True:
                            old_config = tree.vertices[node_tree_id];
                            node_tree_id = tree.edges[node_tree_id];
                            node_config = tree.vertices[node_tree_id];
                            self.planning_env.PlotEdge(old_config, node_config);
                            if (node_tree_id == tree.GetRootId()):
                                plan_length = self.Plan_Length(plan)
                                print("--- %s seconds ---" % (time.time() - start_time))
                                print("--- %s plan length ---" % plan_length)
                                print("--- %s vertices ---" % len(tree.vertices))
                                return plan
                            else:
                                plan.insert(1, node_config)
        return plan

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

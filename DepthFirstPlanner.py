from Queue import LifoQueue
import numpy
import time

class DepthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()

    def Plan(self, start_config, goal_config):

        self.planning_env.InitializePlot( goal_config)
        
        start_time = time.time()
        plan = []
        
        # TODO: Here you will implement the depth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        plan.append(start_config)
        plan.append(goal_config)
        nodes_stack = LifoQueue()
        start_id=self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        nodes_stack.put(start_id)
        end_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        curr_id=start_id
        seen=[start_id]
        while curr_id!=end_id:
            successors = self.planning_env.GetSuccessors(curr_id)
            for i in range(0,numpy.size(successors,0)):
                # print "succesor"+str(self.planning_env.discrete_env.NodeIdToConfiguration(successors[i]))
                # print "grid"+str(self.planning_env.discrete_env.NodeIdToGridCoord(successors[i]))
                # print "from depth"+str(successors)
                # print "from depth single"+str(successors[i])
                #print seen
                if successors[i] in seen:
                    continue
                else:
                    nodes_stack.put(successors[i]) 
                    # print nodes_stack
            node_id=nodes_stack.get()
            self.nodes[node_id]=curr_id
            
            curr_id=node_id
            seen.append(curr_id)
        plan_id=curr_id
        while plan_id != start_id:
            plan_config = self.planning_env.discrete_env.NodeIdToConfiguration(plan_id);
            successor_id=self.nodes.get(plan_id)
            successor_config = self.planning_env.discrete_env.NodeIdToConfiguration(successor_id);
            self.planning_env.PlotEdge(plan_config, successor_config);
            plan.insert(1,self.planning_env.discrete_env.NodeIdToConfiguration(successor_id))
            plan_id=successor_id
            #raw_input('Press any key to begin planning')
        
        plan_time = time.time()-start_time
        number_of_nodes = numpy.size(plan,0)
        plan_length = self.Plan_Length(plan)
        print '---- DFS Stats ----'
        print 'Plan Time: ' + str(plan_time)
        print 'Plan Length: ' + str(plan_length)
        print 'Num of Nodes: ' + str(number_of_nodes)
        # self.planning_env.PlotAll(plan)
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

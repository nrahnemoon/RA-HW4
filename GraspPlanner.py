import logging, numpy, openravepy

class GraspPlanner(object):

    def __init__(self, robot, base_planner, arm_planner):
        self.robot = robot
        self.base_planner = base_planner
        self.arm_planner = arm_planner

            
    def GetBasePoseForObjectGrasp(self, obj):

        # Load grasp database
        gmodel = openravepy.databases.grasping.GraspingModel(self.robot, obj)
        if not gmodel.load():
            gmodel.autogenerate()

        base_pose = None
        grasp_config = None
       
        ###################################################################
        # TODO: Here you will fill in the function to compute
        #  a base pose and associated grasp config for the 
        #  grasping the bottle
        ###################################################################
        
        # Order the grasps
        self.grasps = self.gmodel.grasps
        self.graspindices = self.gmodel.graspindices
        self.order_grasps()

        # Generate the Inverse Reachability model
        self.irmodel = openravepy.databases.inversereachability.InverseReachabilityModel(self.robot)
        if not self.irmodel.load():
            self.irmodel.autogenerate()

        # Store initial pose of the robot
        startTransform = self.robot.GetTransform()
        startJointAngles = self.robot.GetActiveDOFValues()

        # Iterate through the sorted grasps, and find a valid base pose corresponding to the best possible grasp
        flag = True
        for grasp in self.orderedGrasps:
        	
        	# Get position and orientation of the hand for the current grasp
        	Tgrasp = self.gmodel.getGlobalGraspTransform(grasp, collisionfree = True)
        	if Tgrasp == None:
        		continue

        	# Choose 10 random base positions from which the robot can reach the object with the current grasp
        	desnityfn, samplerfn, bounds = self.irmodel.computeBaseDistribution(Tgrasp)
        	pose, jointstate = samplerfn(10)

        	# Among the randomly selected base positions, check if there exists a collision-free one from which the robot can extend its arm 
        	# towards the object with the current grasp without collision
        	for i in range(10):
        		# Snap the robot to the centre of the grid it is located in currently, and verify inverse kinematics and collision
        		self.robot.SetTransform(pose[i])
        		curTransform = self.robot.GetTransform()
        		curTransformXYT = [curTransform[0][3], curTransform[1][3], numpy.arctan2(curTransform[1][0], curTransform[0][0])]
        		curGridCoord = self.base_planner.planning_env.discrete_env.ConfigurationToGridCoord(curTransformXYT)
        		curDiscreteTransform = self.base_planner.planning_env.discrete_env.GridCoordToConfiguration(curGridCoord)
        		X = curDiscreteTransform[0]
        		Y = curDiscreteTransform[1]
        		T = curDiscreteTransform[2]
        		curTransform = numpy.array([[numpy.cos(T),	-numpy.sin(T),	0,	X],
        									[numpy.sin(T),	numpy.cos(T),	0,	Y],
        									[0,				0,				1,	0]
        									[0,				0,				0,	1]])
        		self.robot.SetTransform(curTransform)

        		jointAngles = self.robot.GetActiveManipulator().FindIKSolution(T_grasp, filteroptions = 1)
        		if jointAngles == None:
        			continue
    			self.robot.SetActiveDOFValues(jointAngles)

    			if not ( self.robot.GetEnv().CheckCollision(self.robot.GetEnv.GetBodies()[0], self.robot.GetEnv.GetBodies()[1]) or self.robot.CheckSelfCollision() ):
    				flag = False
    				base_pose = numpy.array(curTransform)
    				grasp_config = jointAngles
    				break

			if not flag:
				break

		self.Robot.SetActiveDOFValues(startJointAngles)
		self.Robot.SetTransform(startTransform)

        return base_pose, grasp_config

    def PlanToGrasp(self, obj):

        # Next select a pose for the base and an associated ik for the arm
        base_pose, grasp_config = self.GetBasePoseForObjectGrasp(obj)

        if base_pose is None or grasp_config is None:
            print 'Failed to find solution'
            exit()

        # Now plan to the base pose
        start_pose = numpy.array(self.base_planner.planning_env.herb.GetCurrentConfiguration())
        base_plan = self.base_planner.Plan(start_pose, base_pose)
        base_traj = self.base_planner.planning_env.herb.ConvertPlanToTrajectory(base_plan)

        print 'Executing base trajectory'
        self.base_planner.planning_env.herb.ExecuteTrajectory(base_traj)

        # Now plan the arm to the grasp configuration
        start_config = numpy.array(self.arm_planner.planning_env.herb.GetCurrentConfiguration())
        arm_plan = self.arm_planner.Plan(start_config, grasp_config)
        arm_traj = self.arm_planner.planning_env.herb.ConvertPlanToTrajectory(arm_plan)

        print 'Executing arm trajectory'
        self.arm_planner.planning_env.herb.ExecuteTrajectory(arm_traj)

        # Grasp the bottle
        task_manipulation = openravepy.interfaces.TaskManipulation(self.robot)
        task_manipultion.CloseFingers()
    
    def order_grasps(self):
    	self.orderedGrasps = self.grasps.copy()

    	# Define metrics (1. Minimum singular value, 2. Ratio of sminimum and maximum singular values, 3. Volume of convex hull)
    	SVs = []
    	Ratios = []
    	Volumes =  []

    	# Rate each grasp
    	for grasp in orderedGrasps:
    		rating = self.eval_grasp(grasp)
    		SVs.append(rating[0])
    		Ratios.append(rating[1])
    		Volumes.append(rating[2])

    	# Normalize the minimum singular value ratings
    	ll = min(SVs)
    	ul = max(SVs)
    	SVsNorm = [ (ele - ll)/(ul - ll) for ele in SVs ]

    	# Normalize the volume ratings
    	ll = min(Volumes)
    	ul = max(Volumes)
    	VolumesNorm = [ (ele - ll)/(ul - ll) for ele in Volumes ]

    	# Compute total ratings for the grasps
    	for i, grasp in self.orderedGrasps:
    		grasp[self.graspindices.get('performance')[0]] = SVsNorm[i] + Ratios[i] + VolumesNorm[i]

		# Sort in descending order of performance
		order = numpy.argsort(self.orderedGrasps[:, self.graspindices.get('performance')[0]])
        order = order[::-1]
        self.orderedGrasps = self.orderedGrasps[order]

	def eval_grasp(self, grasp):
		with self.robot:
			try:
				contacts, finalconfig, mindist, volume = self.gmodel.testGrasp(grasp = grasp, translate = True, forceclosure = False)
				bottlePos = self.gmodel.target.GetTransform()[0:3, 3]

				# Define the wrench matrix
				W = []

				for ele in contacts:
					position	= ele[0:3] - bottlePos
					direction 	= -ele[3:]
					direction = numpy.array(direction)
					W_temp = numpy.array([direction, numpy.cross(position, direction)])
					W = numpy.append(W, W_temp)

				W = W.reshape(len(W)/6, 6)
				W = W.transpose()

				# Find the 
				try:
					u, s, v = numpy.linalg.svd(W)
					min_sv = min(s)
					max_sv = max(s)
					ratio = min_sv / max_sv
					volume = math.sqrt(abs(numpy.linalg.det(numpy.dot(W, numpy.transpose(W)))))
					rating = (min_sv, ratio, volume)
					return rating

				except numpy.linalg.linalg.LinAlgError:
					return (0.0, 0.0, 0.0)

			except openravepy.planning_error:
				return (0.0, 0.0, 0.0)

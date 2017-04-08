import logging, numpy, openravepy
import time

class GraspPlanner(object):

    def __init__(self, robot, base_planner, arm_planner):
        self.robot = robot
        self.base_planner = base_planner
        self.arm_planner = arm_planner
            
    def GetBasePoseForObjectGrasp(self, obj):

        # Load grasp database
        self.gmodel = openravepy.databases.grasping.GraspingModel(self.robot, obj)
        if not self.gmodel.load():
            self.gmodel.autogenerate()

        base_pose = None
        grasp_config = None
       
        ###################################################################
        # TODO: Here you will fill in the function to compute
        #  a base pose and associated grasp config for the 
        #  grasping the bottle
        ###################################################################
        
        self.graspindices = self.gmodel.graspindices
        self.grasps = self.gmodel.grasps
        self.order_grasps()
        self.show_grasp(self.grasps_ordered[0])
        graspTransform = self.gmodel.getGlobalGraspTransform(self.grasps_ordered[0], collisionfree = True)

        irmodel = openravepy.databases.inversereachability.InverseReachabilityModel(robot = self.robot)
        irmodel.load()
        densityfn, samplerfn, bounds = irmodel.computeBaseDistribution(graspTransform)

        poses, jointstate = samplerfn(100)
        self.manip = self.robot.GetActiveManipulator()
        start_pose = self.robot.GetTransform()
        handles = []
        init_config = self.base_planner.planning_env.herb.GetCurrentConfiguration()

        for pose in poses:
            self.robot.SetTransform(pose)
            angle = openravepy.axisAngleFromQuat(pose)
            newpose = numpy.array([pose[4], pose[5], angle[2]])
            node = self.base_planner.planning_env.discrete_env.ConfigurationToNodeId(newpose)
            discrete_pose = self.base_planner.planning_env.discrete_env.NodeIdToConfiguration(node)

            handles.append(self.robot.GetEnv().plot3(points=numpy.array(numpy.append(discrete_pose[:2], [0])), pointsize = 15.0))

            quat = openravepy.quatFromAxisAngle([0, 0, discrete_pose[2]])
            quat = numpy.append(quat, discrete_pose[0])
            quat = numpy.append(quat, discrete_pose[1])
            quat = numpy.append(quat, 0)
            self.robot.SetTransform(quat)

            self.base_planner.planning_env.herb.SetCurrentConfiguration(discrete_pose)
            obs = self.robot.GetEnv().GetBodies()
            table = obs[1]

            grasp_config = self.manip.FindIKSolution(graspTransform, filteroptions = openravepy.IkFilterOptions.CheckEnvCollisions.IgnoreEndEffectorCollisions)
            if not self.robot.GetEnv().CheckCollision(self.robot, table) and not grasp_config == None:
                print grasp_config
                self.base_planner.planning_env.herb.SetCurrentConfiguration(init_config)
                return discrete_pose, grasp_config

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
        arm_plan = self.arm_planner.Plan(start_config, grasp_config, epsilon=0.1)
        arm_traj = self.arm_planner.planning_env.herb.ConvertPlanToTrajectory(arm_plan)

        print 'Executing arm trajectory'
        self.arm_planner.planning_env.herb.ExecuteTrajectory(arm_traj)

        # Grasp the bottle
        task_manipulation = openravepy.interfaces.TaskManipulation(self.robot)
        task_manipulation.CloseFingers()
    
    # order the grasps - call eval grasp on each, set the 'performance' index, and sort
    def order_grasps(self):
        self.grasps_ordered = self.grasps.copy() #you should change the order of self.grasps_ordered
        i = 0
        for grasp in self.grasps_ordered:
          print('Evaluating grasp %d of %d' % (i, len(self.grasps_ordered)) )
          grasp[self.graspindices.get('performance')] = self.eval_grasp(grasp)
          i += 1

        # sort!
        order = numpy.argsort(self.grasps_ordered[:,self.graspindices.get('performance')[0]])
        order = order[::-1]
        self.grasps_ordered = self.grasps_ordered[order]
  
    # order the grasps - but instead of evaluating the grasp, evaluate random perturbations of the grasp 
    def order_grasps_noisy(self):
        self.grasps_ordered_noisy = self.grasps.copy()

        # Order grasps based on std deviation in eval score due to uncertainty
        i, NUM_RAND_GRASPS, grasps_noisy_scores, grasps_raw_scores = 0, 5, [], []
        for grasp in self.grasps_ordered_noisy:
          print('Evaluating grasp %d of %d with uncertainty' % (i, len(self.grasps_ordered_noisy)) )
          grasps_raw_scores.append(self.eval_grasp(grasp))     
          rand_grasps =  [self.sample_random_grasp(grasp) for j in range(NUM_RAND_GRASPS)]
          rand_grasps_scores = [self.eval_grasp(rand_grasp) for rand_grasp in rand_grasps]    
          grasps_noisy_scores.append(1 - numpy.std(rand_grasps_scores))
          i += 1

        # Normalize the two metrics
        grasps_raw_scores_sum, grasps_noisy_scores_sum = sum(grasps_raw_scores), sum(grasps_noisy_scores)
        grasps_raw_scores = [score/grasps_raw_scores_sum for score in grasps_raw_scores]
        grasps_noisy_scores = [score/grasps_noisy_scores_sum for score in grasps_noisy_scores]

        # Update score with a weighted mean
        raw_scores_w, noisy_scores_w = 0.5, 0.5
        for i in range(len(self.grasps_ordered_noisy)):
          print('Compute final score %d of %d from individual scores %f, %f' % (i, len(self.grasps_ordered_noisy), grasps_raw_scores[i], grasps_noisy_scores[i]))
          self.grasps_ordered_noisy[i][self.graspindices.get('performance')] = [raw_scores_w*grasps_raw_scores[i] + noisy_scores_w*grasps_noisy_scores[i]]
         
        # sort!
        order = numpy.argsort(self.grasps_ordered_noisy[:,self.graspindices.get('performance')[0]])
        order = order[::-1]
        self.grasps_ordered_noisy = self.grasps_ordered_noisy[order]

    # function to evaluate grasps
    # returns a score, which is some metric of the grasp
    # higher score should be a better grasp
    def eval_grasp(self, grasp):
        with self.robot:
          #contacts is a 2d array, where contacts[i,0-2] are the positions of contact i and contacts[i,3-5] is the direction
          try:
            contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=False)

            obj_position = self.gmodel.target.GetTransform()[0:3,3]
            # for each contact
            G = numpy.zeros((6, len(contacts))) #the wrench matrix
            for i, c in enumerate(contacts):
              pos = c[0:3] - obj_position
              dir = -c[3:] #this is already a unit vector
              
              torque = numpy.cross(pos, dir);

              G[:,i] = numpy.concatenate([dir, torque]);
              #TODO fill G
            
            [_, S, _] = numpy.linalg.svd(G)

            # Using minimization of singular values method
            singularValueMin = S[-1]

            # Maximization of the force ellipsoid volume method 
            #maxEllipsoidVol = math.sqrt(numpy.linalg.det(numpy.dot(G, G.transpose())));

            #return maxEllipsoidVol;
            return singularValueMin

          except:
            #you get here if there is a failure in planning
            #example: if the hand is already intersecting the object at the initial position/orientation
            return  0.00 # TODO you may want to change this

    # given grasp_in, create a new grasp which is altered randomly
    # you can see the current position and direction of the grasp by:
    # grasp[self.graspindices.get('igrasppos')]
    # grasp[self.graspindices.get('igraspdir')]
    def sample_random_grasp(self, grasp_in, dist_sigma=.01, angle_sigma=numpy.pi/24):
        grasp = grasp_in.copy()

        #sample random position
        RAND_DIST_SIGMA = dist_sigma #TODO you may want to change this
        pos_orig = grasp[self.graspindices['igrasppos']]

        grasp[self.graspindices['igrasppos']] = [numpy.random.normal(item, RAND_DIST_SIGMA, 1)[0] for item in grasp[self.graspindices['igrasppos']]]

        #sample random orientation
        RAND_ANGLE_SIGMA = angle_sigma#TODO you may want to change this
        dir_orig = grasp[self.graspindices['igraspdir']]
        roll_orig = grasp[self.graspindices['igrasproll']]
        grasp[self.graspindices['igraspdir']] = [numpy.random.normal(item, RAND_ANGLE_SIGMA, 1)[0] for item in grasp[self.graspindices['igraspdir']]]
        grasp[self.graspindices['igrasproll']] = [numpy.random.normal(item, RAND_ANGLE_SIGMA, 1)[0] for item in grasp[self.graspindices['igrasproll']]]

        return grasp

    #displays the grasp
    def show_grasp(self, grasp, delay=1.5):
        with openravepy.RobotStateSaver(self.gmodel.robot):
          with self.gmodel.GripperVisibility(self.gmodel.manip):
            time.sleep(0.1) # let viewer update?
            try:
              with self.robot.GetEnv():
                contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp, translate=True, forceclosure=True)
                #if mindist == 0:
                #  print 'grasp is not in force closure!'
                contactgraph = self.gmodel.drawContacts(contacts) if len(contacts) > 0 else None
                self.gmodel.robot.GetController().Reset(0)
                self.gmodel.robot.SetDOFValues(finalconfig[0])
                self.gmodel.robot.SetTransform(finalconfig[1])
                self.robot.GetEnv().UpdatePublishedBodies()
                time.sleep(delay)
            except openravepy.planning_error,e:
              print 'bad grasp!',e

# -*- coding: utf-8 -*-
"""
Created on Tue May  1 17:42:03 2018

@author: jack
"""
import time, os, math

from pybullet import *
import pybullet_data
from PID import *
from Rotations import *
from params import *
from logging import *


class PYBULLET(object):
    """Pybullet Simulator.Uses .urdf to generate the arm and scene, uses .obj file to generate shapes.
    The majority of tuned parameters can be found in this class"""
    def __init__(self, experimentnum, optimisationNum, sharedParams, params, fitness):
        self._params = params
        self._fitness = fitness
        self._sharedParams = sharedParams
        self._physics_engine = "PyBullet"
        self._task = experimentnum
        self._current_iteration = optimisationNum

        self._pid = [PID(self._params.get_maxVelocity_joint1()),PID(self._params.get_maxVelocity_joint2()),PID(self._params.get_maxVelocity_joint3()),PID(self._params.get_maxVelocity_joint4()),PID(self._params.get_maxVelocity_joint5()),PID(self._params.get_maxVelocity_joint6())]
        self._linearVelocity = [0,0,0,0,0,0,0,0,0]
        self._theta = [0,0,0,0,0,0,0,0,0]
        self._compare=[]
        self._convertdeg2rad = 57.295779578552
        self._num_steps = 0

        #Setup PyBullet
        self._physicsClient = connect(DIRECT)#or DIRECT for non-graphical version GUI
        setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        setGravity(0,0,-9.81)
        #below moves the camera to a desired position/roll/pitch/yaw
        resetDebugVisualizerCamera( cameraDistance=.4, cameraYaw=0, cameraPitch=5, cameraTargetPosition=[0.3,-0.5,.3])
        configureDebugVisualizer(COV_ENABLE_DEPTH_BUFFER_PREVIEW,enable=0)
        configureDebugVisualizer(COV_ENABLE_SEGMENTATION_MARK_PREVIEW,enable=0)
        configureDebugVisualizer(COV_ENABLE_RGB_BUFFER_PREVIEW,enable=0)

        self.generate_scene()
        if self._task == 3 or self._task == 4:
            self.generate_cube()
        if self._task == 5 or self._task == 6:
            self.generate_cylinder()
        if self._task == 7 or self._task == 8:
            self.generate_cone()
        if self._task == 9 or self._task == 10:
            self.generate_cuboid()

        setPhysicsEngineParameter(fixedTimeStep = self._params.get_timeStep())
        masses = [self._params.get_mass_link1(),self._params.get_mass_link2(), self._params.get_mass_link3(), self._params.get_mass_link4(),self._params.get_mass_link5(), self._params.get_mass_link6()]
        max_force = [self._params.get_maxForce_joint1(), self._params.get_maxForce_joint2(), self._params.get_maxForce_joint3(), self._params.get_maxForce_joint4(), self._params.get_maxForce_joint5(), self._params.get_maxForce_joint6()]
        # inertias = [[self._params.get_inertia_link1_x(),self._params.get_inertia_link1_y(), self._params.get_inertia_link1_z()],
        #             [self._params.get_inertia_link2_x(),self._params.get_inertia_link2_y(), self._params.get_inertia_link2_z()],
        #             [self._params.get_inertia_link3_x(),self._params.get_inertia_link3_y(), self._params.get_inertia_link3_z()],
        #             [self._params.get_inertia_link4_x(),self._params.get_inertia_link4_y(), self._params.get_inertia_link4_z()],
        #             [self._params.get_inertia_link5_x(),self._params.get_inertia_link5_y(), self._params.get_inertia_link5_z()],
        #             [self._params.get_inertia_link6_x(),self._params.get_inertia_link6_y(), self._params.get_inertia_link6_z()]]
        inertiaLinks = [1,2,3,4,5,13]
        for jointNum in range(2,8):
            #Arm Joints up to the wrist
            enableJointForceTorqueSensor(self._kinova, jointNum)
            changeDynamics(self._kinova,linkIndex = jointNum, mass = masses[jointNum-2])
            setJointMotorControl2(self._kinova,jointIndex=jointNum,controlMode=VELOCITY_CONTROL, force=max_force[jointNum-2])
        # for link in range(0,6):
        #     changeDynamics(self._kinova,linkIndex = inertiaLinks[link], localInertiaDiagonal= inertias[link])
        for jointNum in range(14,21):
            # Gripper Dynamics
            changeDynamics(self._kinova,linkIndex = jointNum,lateralFriction = self._params.get_friction_gripper())

        if self._sharedParams == False:
            joint_damping = [self._params.get_jointDamping_joint1(),self._params.get_jointDamping_joint2(), self._params.get_jointDamping_joint3(), self._params.get_jointDamping_joint4(),self._params.get_jointDamping_joint5(), self._params.get_jointDamping_joint6()]
            for jointNum in range(2,8):
                #Arm Joints up to the wrist
                changeDynamics(self._kinova,linkIndex = jointNum, jointDamping =joint_damping[jointNum-2], )
            for jointNum in range(14,21):
                # Gripper Dynamics
                changeDynamics(self._kinova,linkIndex = jointNum,lateralFriction = self._params.get_friction_gripper(), spinningFriction=self._params.get_slidingFriction_gripper(), rollingFriction=self._params.get_rollingFriction_gripper(), restitution= self._params.get_restitution_gripper(), angularDamping = self._params.get_angularDamping_gripper(), linearDamping = self._params.get_linearDamping_gripper())

        #Set fingers rigid with Position control
        for jointNum in range(15,21,2):
            # these are finger joints
            setJointMotorControl2(self._kinova,jointIndex=jointNum,controlMode=POSITION_CONTROL, targetPosition=0, force=2000)

    def generate_scene(self,):
        #Setup Floor
        self._planeId = loadURDF("plane.urdf")
        #Robot Arm
        self._kinovaStartPos = [0.0,0.0,0.055]
        my_path = os.path.abspath(os.path.dirname(__file__))
        path = os.path.join(my_path, "kinova_description/urdf/m1n6s300.urdf")
        self._kinova = loadURDF(path,self._kinovaStartPos,useFixedBase=1)
        if self._sharedParams == True:
            changeDynamics(self._planeId,-1,lateralFriction = self._params.get_friction_floor())
        else:
            changeDynamics(self._planeId,-1,lateralFriction = self._params.get_friction_floor(), spinningFriction=self._params.get_slidingFriction_floor(), rollingFriction=self._params.get_rollingFriction_floor(), restitution= self._params.get_restitution_floor(), angularDamping = self._params.get_angularDamping_floor(), linearDamping = self._params.get_linearDamping_floor())

    def generate_cube(self,):
        """generates a Cube shaped object based upon the supplied .obj file, places at start position and orientation, sets dynamic properties provided by gp_minimise
           The cube is set  with a mass for plastic in experiment 3 and wood for experiment 4.
        """
        self._visualShapeId = -1
        self._cubeStartPos = [0.6,0.0,0.0375]
        self._cubeStartOrientation = getQuaternionFromEuler([0.0,0.0,0.0])
        createMultiBody(0,0)
        my_path = os.path.abspath(os.path.dirname(__file__))
        path = os.path.join(my_path, "kinova_description/Shapes/Cube.obj")
        self._colCubeId = createCollisionShape(GEOM_MESH,fileName=path)
        if self._task == 3:
            self._objectUid = createMultiBody(self._params.get_mass_cubeP(),self._colCubeId,self._visualShapeId,self._cubeStartPos,self._cubeStartOrientation)
            if self._sharedParams == True:
                changeDynamics(self._objectUid,-1,lateralFriction = self._params.get_friction_plastic())
            elif self._sharedParams == False:
                changeDynamics(self._objectUid,-1,lateralFriction = self._params.get_friction_plastic(), spinningFriction=self._params.get_slidingFriction_plastic(), rollingFriction=self._params.get_rollingFriction_plastic(), restitution= self._params.get_restitution_plastic(), angularDamping = self._params.get_angularDamping_plastic(), linearDamping = self._params.get_linearDamping_plastic())
        elif self._task == 4:
            self._objectUid = createMultiBody(self._params.get_mass_cubeW(),self._colCubeId,self._visualShapeId,self._cubeStartPos,self._cubeStartOrientation)
            if self._sharedParams == True:
                changeDynamics(self._objectUid,-1,lateralFriction = self._params.get_friction_wood())
            elif self._sharedParams == False:
                changeDynamics(self._objectUid,-1,lateralFriction = self._params.get_friction_wood(), spinningFriction=self._params.get_slidingFriction_wood(), rollingFriction=self._params.get_rollingFriction_wood(), restitution= self._params.get_restitution_wood(), angularDamping = self._params.get_angularDamping_wood(), linearDamping = self._params.get_linearDamping_wood())
        

    def generate_cylinder(self,):
        """generates a cylinder shaped object based upon the supplied .obj file, places at start position and orientation, sets dynamic properties provided by gp_minimise
           The cylinder is set  with a mass for plastic in experiment 5 and wood for experiment 6.
        """
        self._visualShapeId = -1
        self._cubeStartPos = [0.6,0.0,0.0375]
        self._cubeStartOrientation = getQuaternionFromEuler([1.571,0,0])
        createMultiBody(0,0)
        my_path = os.path.abspath(os.path.dirname(__file__))
        path = os.path.join(my_path, "kinova_description/Shapes/Cylinder.obj")
        self._colCubeId = createCollisionShape(GEOM_MESH,fileName=path)
        if self._task == 5:
            self._objectUid = createMultiBody(self._params.get_mass_cylinderP(),self._colCubeId,self._visualShapeId,self._cubeStartPos,self._cubeStartOrientation)
            if self._sharedParams == True:
                changeDynamics(self._objectUid,-1,lateralFriction = self._params.get_friction_plastic())
            elif self._sharedParams == False:
                changeDynamics(self._objectUid,-1,lateralFriction = self._params.get_friction_plastic(), spinningFriction=self._params.get_slidingFriction_plastic(), rollingFriction=self._params.get_rollingFriction_plastic(), restitution= self._params.get_restitution_plastic(), angularDamping = self._params.get_angularDamping_plastic(), linearDamping = self._params.get_linearDamping_plastic())
        if self._task == 6:
            self._objectUid = createMultiBody(self._params.get_mass_cylinderW(),self._colCubeId,self._visualShapeId,self._cubeStartPos,self._cubeStartOrientation)
            if self._sharedParams == True:
                changeDynamics(self._objectUid,-1,lateralFriction = self._params.get_friction_wood())
            elif self._sharedParams == False:
                changeDynamics(self._objectUid,-1,lateralFriction = self._params.get_friction_wood(), spinningFriction=self._params.get_slidingFriction_wood(), rollingFriction=self._params.get_rollingFriction_wood(), restitution= self._params.get_restitution_wood(), angularDamping = self._params.get_angularDamping_wood(), linearDamping = self._params.get_linearDamping_wood())

    def generate_cone(self,):
        """generates a cone shaped object based upon the supplied .obj file, places at start position and orientation, sets dynamic properties provided by gp_minimise
           The cone is set  with a mass for plastic in experiment 7 and wood for experiment 8.
        """
        self._visualShapeId = -1
        self._cubeStartPos = [0.6,0.00,0.0375]
        self._cubeStartOrientation = getQuaternionFromEuler([0.0,1.92957,0.0])
        createMultiBody(0,0)
        my_path = os.path.abspath(os.path.dirname(__file__))
        path = os.path.join(my_path, "kinova_description/Shapes/Cone.obj")
        self._colCubeId = createCollisionShape(GEOM_MESH,fileName=path)
        if self._task == 7:
            self._objectUid = createMultiBody(self._params.get_mass_coneP(),self._colCubeId,self._visualShapeId,self._cubeStartPos,self._cubeStartOrientation)
            if self._sharedParams == True:
                changeDynamics(self._objectUid,-1,lateralFriction = self._params.get_friction_plastic())
            elif self._sharedParams == False:
                changeDynamics(self._objectUid,-1,lateralFriction = self._params.get_friction_plastic(), spinningFriction=self._params.get_slidingFriction_plastic(), rollingFriction=self._params.get_rollingFriction_plastic(), restitution= self._params.get_restitution_plastic(), angularDamping = self._params.get_angularDamping_plastic(), linearDamping = self._params.get_linearDamping_plastic())
        if self._task == 8:
            self._objectUid = createMultiBody(self._params.get_mass_coneW(),self._colCubeId,self._visualShapeId,self._cubeStartPos,self._cubeStartOrientation)
            if self._sharedParams == True:
                changeDynamics(self._objectUid,-1,lateralFriction = self._params.get_friction_wood())
            elif self._sharedParams == False:
                changeDynamics(self._objectUid,-1,lateralFriction = self._params.get_friction_wood(), spinningFriction=self._params.get_slidingFriction_wood(), rollingFriction=self._params.get_rollingFriction_wood(), restitution= self._params.get_restitution_wood(), angularDamping = self._params.get_angularDamping_wood(), linearDamping = self._params.get_linearDamping_wood())

    def generate_cuboid(self,):
        """generates a cuboid shaped object based upon the supplied .obj file, places at start position and orientation, sets dynamic properties provided by gp_minimise
           The cuboid is set  with a mass for plastic in experiment 9 and wood for experiment 10.
        """
        self._visualShapeId = -1
        self._cubeStartPos = [0.6,0.0,0.075]
        self._cubeStartOrientation = getQuaternionFromEuler([0.0,0.0,0.0])
        createMultiBody(0,0)
        my_path = os.path.abspath(os.path.dirname(__file__))
        path = os.path.join(my_path, "kinova_description/Shapes/Cuboid.obj")
        self._colCubeId = createCollisionShape(GEOM_MESH,fileName=path)
        if self._task == 9:
            self._objectUid = createMultiBody(self._params.get_mass_cuboidP(),self._colCubeId,self._visualShapeId,self._cubeStartPos,self._cubeStartOrientation)
            if self._sharedParams == True:
                changeDynamics(self._objectUid,-1,lateralFriction = self._params.get_friction_plastic())
            elif self._sharedParams == False:
                changeDynamics(self._objectUid,-1,lateralFriction = self._params.get_friction_plastic(), spinningFriction=self._params.get_slidingFriction_plastic(), rollingFriction=self._params.get_rollingFriction_plastic(), restitution= self._params.get_restitution_plastic(), angularDamping = self._params.get_angularDamping_plastic(), linearDamping = self._params.get_linearDamping_plastic())
        if self._task == 10:
            self._objectUid = createMultiBody(self._params.get_mass_cuboidW(),self._colCubeId,self._visualShapeId,self._cubeStartPos,self._cubeStartOrientation)
            if self._sharedParams == True:
                changeDynamics(self._objectUid,-1,lateralFriction = self._params.get_friction_wood())
            elif self._sharedParams == False:
                changeDynamics(self._objectUid,-1,lateralFriction = self._params.get_friction_wood(), spinningFriction=self._params.get_slidingFriction_wood(), rollingFriction=self._params.get_rollingFriction_wood(), restitution= self._params.get_restitution_wood(), angularDamping = self._params.get_angularDamping_wood(), linearDamping = self._params.get_linearDamping_wood())

    def set_num_steps(self):
        self._num_steps = simSteps(self._task,self._params.get_timeStep())

    def run_pybullet(self,):
        """Runs this simulation for the number of steps while setting target thetas for joint movements via PID control"""
        self.set_num_steps()

        for simStep in range(self._num_steps):

            self._pid = set_target_thetas(self._num_steps, self._pid,self._task,simStep,self._physics_engine)

            if simStep % int(0.1*(1/self._params.get_timeStep())) == 0:
                for jointNum in range(6):
                    self._theta[jointNum] = getJointState(self._kinova, jointNum)[0]
                    self._linearVelocity[jointNum] = self._pid[jointNum].get_velocity(math.degrees(self._theta[jointNum]))/self._convertdeg2rad
                    setJointMotorControl2(bodyIndex=self._kinova,jointIndex=jointNum,controlMode=VELOCITY_CONTROL,targetVelocity=self._linearVelocity[jointNum])

                    arm_translation = [getLinkState(self._kinova,10,1)[0][0],getLinkState(self._kinova,10,1)[0][1],getLinkState(self._kinova,10,1)[0][2]]
                    arm_rotation = [getLinkState(self._kinova,10,1)[1][3],getLinkState(self._kinova,10,1)[1][0],getLinkState(self._kinova,10,1)[1][1],getLinkState(self._kinova,10,1)[1][2]]
                    arm_torque = [getJointState(self._kinova,2)[3],getJointState(self._kinova,3)[3],getJointState(self._kinova,4)[3],getJointState(self._kinova,5)[3],getJointState(self._kinova,6)[3],getJointState(self._kinova,7)[3]]
                    finger_position = [getJointState(self._kinova,15)[0],getJointState(self._kinova,16)[0],getJointState(self._kinova,17)[0]]
                    force_torque = [getJointState(self._kinova,9)[2][0],getJointState(self._kinova,9)[2][1],getJointState(self._kinova,9)[2][2],getJointState(self._kinova,9)[2][3],getJointState(self._kinova,9)[2][4],getJointState(self._kinova,9)[2][5]]

                if(self._task == 1 or self._task == 2):
                    # There is no shape in experiment 1 or 2
                    self._compare.append(arm_translation+arm_rotation+arm_torque+finger_position+force_torque)
                else:
                    # There is a shape (cube cone cylinder cuboid) in experiments 3-10
                    object_translation = [getBasePositionAndOrientation(self._objectUid)[0][0],getBasePositionAndOrientation(self._objectUid)[0][1],getBasePositionAndOrientation(self._objectUid)[0][2]]
                    object_rotation = [getBasePositionAndOrientation(self._objectUid)[1][3],getBasePositionAndOrientation(self._objectUid)[1][0],getBasePositionAndOrientation(self._objectUid)[1][1],getBasePositionAndOrientation(self._objectUid)[1][2]]
                    self._compare.append(arm_translation + arm_rotation + arm_torque + finger_position + force_torque + object_translation + object_rotation)

            stepSimulation()
            time.sleep(self._params.get_timeStep())

        disconnect()
        return self._fitness.calculateFitness(self._compare, self._task)
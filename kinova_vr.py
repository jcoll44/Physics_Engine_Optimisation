# -*- coding: utf-8 -*-
"""
Created on Mon May 28 14:01:53 2018

@author: jack
"""
from os.path import dirname, join, abspath
from pyrep import PyRep
from pyrep.objects.joint import Joint
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.object import Object
from pyrep.objects.force_sensor import ForceSensor
from pyrep.backend import vrep

import numpy as np
import sys, math, os, cffi
from PID import *
from Rotations import *
from params import *
from logging import *

class VREP(object):
    """Vrep Simulator, uses multiple engines. Uses .ttt file to generate arm and shape.
    The majority of tuned parameters can be found in this class"""
    def __init__(self, experimentnum, optimisationNum, sharedParams, params, fitness):
        self._params = params
        self._fitness = fitness
        self._sharedParams = sharedParams
        self._physics_engine = ""
        self._task = experimentnum
        self._current_iteration = optimisationNum
        self._headless = True


        self._pid = [PID(self._params.get_maxVelocity_joint1()),PID(self._params.get_maxVelocity_joint2()),PID(self._params.get_maxVelocity_joint3()),PID(self._params.get_maxVelocity_joint4()),PID(self._params.get_maxVelocity_joint5()),PID(self._params.get_maxVelocity_joint6())]
        self._linearVelocity = [0,0,0,0,0,0,0,0,0]
        self._theta = [0,0,0,0,0,0,0,0,0]
        self._compare=[]
        self._convertdeg2rad = 57.295779578552
        self._num_steps = 0
        self._arm_joints = [0]*9
        self._arm_shape = [0]*6


        #PyRep API
        self._pr = PyRep()

        if self._task == 1:
            """Task 1 has no shape, only Kinova Mico"""
            self._pr.launch(join(dirname(abspath(__file__)), 'kinova_description/urdf/m1n6s300.ttt'), headless=self._headless)
        if self._task == 2:
            """Task 2 has no shape, only Kinova Mico"""
            self._pr.launch(join(dirname(abspath(__file__)), 'kinova_description/urdf/m1n6s300.ttt'), headless=self._headless)
        if self._task == 3:
            """Task 3 has a respondable plastic cube shape"""
            self._pr.launch(join(dirname(abspath(__file__)), 'kinova_description/urdf/m1n6s300_cube.ttt'), headless=self._headless)
            self._shape = Shape('Shape')
            self._shape.set_mass(self._params.get_mass_cubeP())
            if self._sharedParams == True:
                self._pr.script_call('set_shape_params@Floor',vrep.sim_scripttype_customizationscript,[],[self._params.get_friction_plastic()],[],[])
            else:
                self._pr.script_call('set_shape_params@Floor',vrep.sim_scripttype_customizationscript,[],[self._params.get_friction_plastic(), self._params.get_rollingFriction_plastic(), self._params.get_restitution_plastic(),self._params.get_linearDamping_plastic(), self._params.get_angularDamping_plastic()],[],[])
        if self._task == 4:
            """Task 4 has a respondable wooden cube shape"""
            self._pr.launch(join(dirname(abspath(__file__)), 'kinova_description/urdf/m1n6s300_cube.ttt'), headless=self._headless)
            self._shape = Shape('Shape')
            self._shape.set_mass(self._params.get_mass_cubeW())
            if self._sharedParams == True:
                self._pr.script_call('set_shape_params@Floor',vrep.sim_scripttype_customizationscript,[],[self._params.get_friction_wood()],[],[])
            else:
                self._pr.script_call('set_shape_params@Floor',vrep.sim_scripttype_customizationscript,[],[self._params.get_friction_wood(), self._params.get_rollingFriction_wood(), self._params.get_restitution_wood(),self._params.get_linearDamping_wood(), self._params.get_angularDamping_wood()],[],[])        
        if self._task == 5:
            #plastic
            """Task 5 has a respondable plastic cylinder shape"""
            self._pr.launch(join(dirname(abspath(__file__)), 'kinova_description/urdf/m1n6s300_cylinder.ttt'), headless=self._headless)
            self._shape = Shape('Shape')
            self._shape.set_mass(self._params.get_mass_cylinderP())
            if self._sharedParams == True:
                self._pr.script_call('set_shape_params@Floor',vrep.sim_scripttype_customizationscript,[],[self._params.get_friction_plastic()],[],[])
            else:
                self._pr.script_call('set_shape_params@Floor',vrep.sim_scripttype_customizationscript,[],[self._params.get_friction_plastic(), self._params.get_rollingFriction_plastic(), self._params.get_restitution_plastic(),self._params.get_linearDamping_plastic(), self._params.get_angularDamping_plastic()],[],[])        
        if self._task == 6:
            #wood
            """Task 6 has a respondable wooden cylinder shape"""
            self._pr.launch(join(dirname(abspath(__file__)), 'kinova_description/urdf/m1n6s300_cylinder.ttt'), headless=self._headless)
            self._shape = Shape('Shape')
            self._shape.set_mass(self._params.get_mass_cylinderW())
            if self._sharedParams == True:
                self._pr.script_call('set_shape_params@Floor',vrep.sim_scripttype_customizationscript,[],[self._params.get_friction_wood()],[],[])
            else:
                self._pr.script_call('set_shape_params@Floor',vrep.sim_scripttype_customizationscript,[],[self._params.get_friction_wood(), self._params.get_rollingFriction_wood(), self._params.get_restitution_wood(),self._params.get_linearDamping_wood(), self._params.get_angularDamping_wood()],[],[])
        if self._task == 7:
            #plastic
            """Task 7 has a respondable plastic cone shape, the dummy object is to track its base position"""
            self._pr.launch(join(dirname(abspath(__file__)), 'kinova_description/urdf/m1n6s300_cone.ttt'), headless=self._headless)
            self._shape = Shape('Shape')
            self._shape.set_mass(self._params.get_mass_coneP())
            self._dummy = Dummy('Dummy')
            if self._sharedParams == True:
                self._pr.script_call('set_shape_params@Floor',vrep.sim_scripttype_customizationscript,[],[self._params.get_friction_plastic()],[],[])
            else:
                self._pr.script_call('set_shape_params@Floor',vrep.sim_scripttype_customizationscript,[],[self._params.get_friction_plastic(), self._params.get_rollingFriction_plastic(), self._params.get_restitution_plastic(),self._params.get_linearDamping_plastic(), self._params.get_angularDamping_plastic()],[],[])        
        if self._task == 8:
            #wood
            """Task 8 has a respondable wooden cone shape, the dummy object is to track its base position"""
            self._pr.launch(join(dirname(abspath(__file__)), 'kinova_description/urdf/m1n6s300_cone.ttt'), headless=self._headless)
            self._shape = Shape('Shape')
            self._shape.set_mass(self._params.get_mass_coneW())
            self._dummy = Dummy('Dummy')
            if self._sharedParams == True:
                self._pr.script_call('set_shape_params@Floor',vrep.sim_scripttype_customizationscript,[],[self._params.get_friction_wood()],[],[])
            else:
                self._pr.script_call('set_shape_params@Floor',vrep.sim_scripttype_customizationscript,[],[self._params.get_friction_wood(), self._params.get_rollingFriction_wood(), self._params.get_restitution_wood(),self._params.get_linearDamping_wood(), self._params.get_angularDamping_wood()],[],[])
        if self._task == 9:
            #plastic
            """Task 9 has a respondable plastic cuboid shape"""
            self._pr.launch(join(dirname(abspath(__file__)), 'kinova_description/urdf/m1n6s300_cuboid.ttt'), headless=self._headless)
            self._shape = Shape('Shape')
            self._shape.set_mass(self._params.get_mass_cuboidP())
            if self._sharedParams == True:
                self._pr.script_call('set_shape_params@Floor',vrep.sim_scripttype_customizationscript,[],[self._params.get_friction_plastic()],[],[])
            else:
                self._pr.script_call('set_shape_params@Floor',vrep.sim_scripttype_customizationscript,[],[self._params.get_friction_plastic(), self._params.get_rollingFriction_plastic(), self._params.get_restitution_plastic(),self._params.get_linearDamping_plastic(), self._params.get_angularDamping_plastic()],[],[])        
        if self._task == 10:
            #wood
            """Task 10 has a respondable wooden cuboid shape"""
            self._pr.launch(join(dirname(abspath(__file__)), 'kinova_description/urdf/m1n6s300_cuboid.ttt'), headless=self._headless)
            self._shape = Shape('Shape')
            self._shape.set_mass(self._params.get_mass_cuboidW())
            if self._sharedParams == True:
                self._pr.script_call('set_shape_params@Floor',vrep.sim_scripttype_customizationscript,[],[self._params.get_friction_wood()],[],[])
            else:
                self._pr.script_call('set_shape_params@Floor',vrep.sim_scripttype_customizationscript,[],[self._params.get_friction_wood(), self._params.get_rollingFriction_wood(), self._params.get_restitution_wood(),self._params.get_linearDamping_wood(), self._params.get_angularDamping_wood()],[],[])
        
        ffi = cffi.FFI()
        gravity = np.array((0.0,0.0,-9.81), dtype=np.float32)
        vrep.simSetArrayParameter(vrep.sim_arrayparam_gravity, ffi.cast('float *',gravity.ctypes.data))
        vrep.simSetFloatParameter(vrep.sim_floatparam_simulation_time_step, self._params.get_timeStep())
        self._pr.script_call('set_engine_params@Floor',vrep.sim_scripttype_customizationscript,[],[self._params.get_timeStep()],[],[])

        # Below sets name for arm components and sets tuned parameters
        self._arm_shape[0] = Shape('m1n6s300_link_1_respondable')
        self._arm_shape[1] = Shape('m1n6s300_link_2_respondable')
        self._arm_shape[2] = Shape('m1n6s300_link_3_respondable')
        self._arm_shape[3] = Shape('m1n6s300_link_4_respondable')
        self._arm_shape[4] = Shape('m1n6s300_link_5_respondable')
        self._arm_shape[5] = Shape('m1n6s300_link_6_respondable')
        self._arm_shape[0].set_mass(self._params.get_mass_link1())
        self._arm_shape[1].set_mass(self._params.get_mass_link2())
        self._arm_shape[2].set_mass(self._params.get_mass_link3())
        self._arm_shape[3].set_mass(self._params.get_mass_link4())
        self._arm_shape[4].set_mass(self._params.get_mass_link5())
        self._arm_shape[5].set_mass(self._params.get_mass_link6())
        # self._pr.script_call('set_inertia_params@Floor',vrep.sim_scripttype_customizationscript,[self._arm_shape[0].get_handle()],[self._params.get_mass_link1()],[],[])
        # self._pr.script_call('set_inertia_params@Floor',vrep.sim_scripttype_customizationscript,[self._arm_shape[1].get_handle()],[self._params.get_mass_link2()],[],[])
        # self._pr.script_call('set_inertia_params@Floor',vrep.sim_scripttype_customizationscript,[self._arm_shape[2].get_handle()],[self._params.get_mass_link3()],[],[])
        # self._pr.script_call('set_inertia_params@Floor',vrep.sim_scripttype_customizationscript,[self._arm_shape[3].get_handle()],[self._params.get_mass_link4()],[],[])
        # self._pr.script_call('set_inertia_params@Floor',vrep.sim_scripttype_customizationscript,[self._arm_shape[4].get_handle()],[self._params.get_mass_link5()],[],[])
        # self._pr.script_call('set_inertia_params@Floor',vrep.sim_scripttype_customizationscript,[self._arm_shape[5].get_handle()],[self._params.get_mass_link6()],[],[])
        # Below sets name for arm joints and sets tuned parameters
        self._arm_joints[0] = Joint('m1n6s300_joint_1')
        self._arm_joints[1] = Joint('m1n6s300_joint_2')
        self._arm_joints[2] = Joint('m1n6s300_joint_3')
        self._arm_joints[3] = Joint('m1n6s300_joint_4')
        self._arm_joints[4] = Joint('m1n6s300_joint_5')
        self._arm_joints[5] = Joint('m1n6s300_joint_6')
        self._arm_joints[6] = Joint('m1n6s300_joint_finger_1')
        self._arm_joints[7] = Joint('m1n6s300_joint_finger_2')
        self._arm_joints[8] = Joint('m1n6s300_joint_finger_3')
        self._measure_joint = ForceSensor('gripper_mount_joint_7')
        self._arm_joints[0].set_joint_force(self._params.get_maxForce_joint1())
        self._arm_joints[1].set_joint_force(self._params.get_maxForce_joint2())
        self._arm_joints[2].set_joint_force(self._params.get_maxForce_joint3())
        self._arm_joints[3].set_joint_force(self._params.get_maxForce_joint4())
        self._arm_joints[4].set_joint_force(self._params.get_maxForce_joint5())
        self._arm_joints[5].set_joint_force(self._params.get_maxForce_joint6())

        print(self._sharedParams)
        if self._sharedParams == True:
            self._pr.script_call('set_gripper_params@Floor',vrep.sim_scripttype_customizationscript,[],[self._params.get_friction_gripper()],[],[])
            self._pr.script_call('set_floor_params@Floor',vrep.sim_scripttype_customizationscript,[],[self._params.get_friction_floor()],[],[])
        else:
            self._pr.script_call('set_gripper_params@Floor',vrep.sim_scripttype_customizationscript,[],[self._params.get_friction_gripper(), self._params.get_rollingFriction_gripper(), self._params.get_restitution_gripper(),self._params.get_linearDamping_gripper(), self._params.get_angularDamping_gripper()],[],[])
            self._pr.script_call('set_floor_params@Floor',vrep.sim_scripttype_customizationscript,[],[self._params.get_friction_floor(), self._params.get_rollingFriction_floor(), self._params.get_restitution_floor(),self._params.get_linearDamping_floor(), self._params.get_angularDamping_floor()],[],[])
            # self._pr.script_call('set_joint_params@Floor',vrep.sim_scripttype_customizationscript,[],[self._params.get_normalCFM_joint1(),self._params.get_normalCFM_joint2(),self._params.get_normalCFM_joint3(),self._params.get_normalCFM_joint4(),self._params.get_normalCFM_joint5(),self._params.get_normalCFM_joint6(),self._params.get_stopERP_joint1(),self._params.get_stopERP_joint2(),self._params.get_stopERP_joint3(),self._params.get_stopERP_joint4(),self._params.get_stopERP_joint5(),self._params.get_stopERP_joint6(),self._params.get_stopCFM_joint1(),self._params.get_stopCFM_joint2(),self._params.get_stopCFM_joint3(),self._params.get_stopCFM_joint4(),self._params.get_stopCFM_joint5(),self._params.get_stopCFM_joint6(),self._params.get_bounce_joint1(),self._params.get_bounce_joint2(),self._params.get_bounce_joint3(),self._params.get_bounce_joint4(),self._params.get_bounce_joint5(),self._params.get_bounce_joint6(),],[],[])


    def set_num_steps(self):
        self._num_steps = simSteps(self._task,self._params.get_timeStep())

    def Bullet283(self):
        """Set Physics Engine Bullet283 """
        self._physics_engine = "Bullet283"
        return self.run_simulation(self._task)

    def Bullet278(self):
        """Set Physics Engine Bullet278 """
        vrep.simSetInt32Parameter(vrep.sim_intparam_dynamic_engine,0)
        self._physics_engine = "Bullet278"
        return self.run_simulation(self._task)

    def ODE(self):
        """Set Physics Engine ODE """
        vrep.simSetInt32Parameter(vrep.sim_intparam_dynamic_engine,1)
        self._physics_engine = "ODE"
        return self.run_simulation(self._task)

    def Vortex(self):
        """Set Physics Engine Vortex """
        vrep.simSetInt32Parameter(vrep.sim_intparam_dynamic_engine,2)
        self._physics_engine = "Vortex"
        return self.run_simulation(self._task)

    def Newton(self):
        """Set Physics Engine Newton """
        vrep.simSetInt32Parameter(vrep.sim_intparam_dynamic_engine,3)
        self._physics_engine = "Newton"
        return self.run_simulation(self._task)

    def run_simulation(self,experimentnumber):
        """Runs this simulation for the number of steps while setting target thetas for joint movements via PID control"""
        self.set_num_steps()
        self._pr.start()

        for simStep in range (self._num_steps):

            self._pid = set_target_thetasVR(self._num_steps, self._pid,self._task,simStep,self._physics_engine)


            if simStep % int(0.1*(1/self._params.get_timeStep())) == 0:
                for joint in range(6):
                    #Control of the Arm
                    self._theta[joint] = self._arm_joints[joint].get_joint_position()
                    self._linearVelocity[joint] = self._pid[joint].get_velocity(math.degrees(self._theta[joint]))/self._convertdeg2rad
                    self._arm_joints[joint].set_joint_target_velocity(self._linearVelocity[joint])

                #Saving arm stats
                arm_translation = self._measure_joint.get_position()
                arm_rotation = [self._measure_joint.get_quaternion()[3]]+self._measure_joint.get_quaternion()[0:3]
                try:
                    arm_torque = [self._arm_joints[0].get_joint_force(),self._arm_joints[1].get_joint_force(),self._arm_joints[2].get_joint_force(),self._arm_joints[3].get_joint_force(),self._arm_joints[4].get_joint_force(),self._arm_joints[5].get_joint_force()]
                except:
                    arm_torque = [0]*6
                finger_position = [self._arm_joints[6].get_joint_position(),self._arm_joints[7].get_joint_position(),self._arm_joints[8].get_joint_position()]
                force_torque = self._measure_joint.read()[0]+self._measure_joint.read()[1]

                if(experimentnumber == 1 or experimentnumber == 2):
                    self._compare.append(arm_translation+arm_rotation+arm_torque+finger_position+force_torque)

                elif(experimentnumber == 7 or experimentnumber == 8):
                    object_translation = self._dummy.get_position()
                    object_rotation = [self._dummy.get_quaternion()[3]]+self._dummy.get_quaternion()[0:3]
                    self._compare.append(arm_translation + arm_rotation + arm_torque + finger_position + force_torque + object_translation + object_rotation)
                else:
                    object_translation = self._shape.get_position()
                    object_rotation = [self._shape.get_quaternion()[3]]+self._shape.get_quaternion()[0:3]
                    self._compare.append(arm_translation + arm_rotation + arm_torque + finger_position + force_torque + object_translation + object_rotation)

            #Stepping simulator
            self._pr.step()

        self._pr.stop()
        self._pr.shutdown()

        return self._fitness.calculateFitness(self._compare, self._task)



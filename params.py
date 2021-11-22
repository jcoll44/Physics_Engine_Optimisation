# -*- coding: utf-8 -*-
"""
Created on Fri Aug 23 12:25:51 2019

@author: jack
"""

weightLower = 0.7
weightUpper = 1.3
inertiaLower = 0.5
inertiaUpper = 1.5

class PARAMS(object):


    def __init__(self,sharedParams):
        """Initialise parameter object that is passed to gp_minimise. Values must be floats(Real) or ints(Integer)
        ranges can be modified to suit user **Note gp_minimise does support Categorical lists, not used here"""
        self._sharedParams = sharedParams

        self.timeStep = (0.001,0.05)
        self.mass_link1 = (0.7477*weightLower,0.7477*weightUpper)
        self.mass_link2 = (0.7477*weightLower,0.7477*weightUpper)
        self.mass_link3 = (0.606*weightLower,0.606*weightUpper)
        self.mass_link4 = (0.1785*weightLower,0.1785*weightUpper)
        self.mass_link5 = (0.1785*weightLower,0.1785*weightUpper)
        self.mass_link6 = (0.727*weightLower,0.727*weightUpper)
        self.mass_cubeP = (0.07*weightLower, 0.07*weightUpper)
        self.mass_cubeW = (0.46*weightLower,0.46*weightUpper)
        self.mass_cylinderP = (0.12*weightLower,0.12*weightUpper)
        self.mass_cylinderW = (0.78*weightLower,0.78*weightUpper)
        self.mass_coneP = (0.04*weightLower,0.04*weightUpper)
        self.mass_coneW = (0.17*weightLower,0.17*weightUpper)
        self.mass_cuboidP = (0.14*weightLower,0.14*weightUpper)
        self.mass_cuboidW = (0.85*weightLower, 0.85*weightUpper)
        # self.inertia_link1_x = (0.00152031725204*inertiaLower, 0.00152031725204*inertiaUpper)
        # self.inertia_link1_y = (0.00152031725204*inertiaLower, 0.00152031725204*inertiaUpper)
        # self.inertia_link1_z = (0.00059816*inertiaLower, 0.00059816*inertiaUpper)
        # self.inertia_link2_x = (0.00467090931629*inertiaLower, 0.00467090931629*inertiaUpper)
        # self.inertia_link2_y = (0.000386856*inertiaLower, 0.000386856*inertiaUpper)
        # self.inertia_link2_z = (0.00467090931629*inertiaLower, 0.00467090931629*inertiaUpper)
        # self.inertia_link3_x = (0.0003837984648*inertiaLower, 0.0003837984648*inertiaUpper)
        # self.inertia_link3_y = (0.0001212*inertiaLower, 0.0001212*inertiaUpper)
        # self.inertia_link3_z = (-0.0003837984648*inertiaLower, 0.0003837984648*inertiaUpper)
        # self.inertia_link4_x = (0.00007734969*inertiaLower, 0.00007734969*inertiaUpper)
        # self.inertia_link4_y = (0.00007734969*inertiaLower, 0.00007734969*inertiaUpper)
        # self.inertia_link4_z = (0.0001428*inertiaLower, 0.0001428*inertiaUpper)
        # self.inertia_link5_x = (0.00007734969*inertiaLower, 0.00007734969*inertiaUpper)
        # self.inertia_link5_y = (0.00007734969*inertiaLower, 0.00007734969*inertiaUpper)
        # self.inertia_link5_z = (0.0001428*inertiaLower, 0.0001428*inertiaUpper)
        # self.inertia_link6_x = (0.0003453236187*inertiaLower, 0.0003453236187*inertiaUpper)
        # self.inertia_link6_y = (0.0003453236187*inertiaLower, 0.0003453236187*inertiaUpper)
        # self.inertia_link6_z = (0.0005816*inertiaLower, 0.0005816*inertiaUpper)
            #Joint
        self.maxForce_joint1 = (100,9000)
        self.maxForce_joint2 = (100,9000)
        self.maxForce_joint3 = (100,9000)
        self.maxForce_joint4 = (100,9000)
        self.maxForce_joint5 = (100,9000)
        self.maxForce_joint6 = (100,9000)
        self.maxVelocity_joint1 = (10,40)
        self.maxVelocity_joint2 = (10,40)
        self.maxVelocity_joint3 = (10,40)
        self.maxVelocity_joint4 = (10,40)
        self.maxVelocity_joint5 = (10,40)
        self.maxVelocity_joint6 = (10,40)
        self.jointDamping_joint1 = (0.0001,0.9)
        self.jointDamping_joint2 = (0.0001,0.9)
        self.jointDamping_joint3 = (0.0001,0.9)
        self.jointDamping_joint4 = (0.0001,0.9)
        self.jointDamping_joint5 = (0.0001,0.9)
        self.jointDamping_joint6 = (0.0001,0.9)
        # self.stopERP_joint1 = (0.0001,0.9)
        # self.stopERP_joint2 = (0.0001,0.9)
        # self.stopERP_joint3 = (0.0001,0.9)
        # self.stopERP_joint4 = (0.0001,0.9)
        # self.stopERP_joint5 = (0.0001,0.9)
        # self.stopERP_joint6 = (0.0001,0.9)
        # self.normalCFM_joint1 = (0.0001,0.9)
        # self.normalCFM_joint2 = (0.0001,0.9)
        # self.normalCFM_joint3 = (0.0001,0.9)
        # self.normalCFM_joint4 = (0.0001,0.9)
        # self.normalCFM_joint5 = (0.0001,0.9)
        # self.normalCFM_joint6 = (0.0001,0.9)
        # self.stopCFM_joint1 = (0.0001,0.9)
        # self.stopCFM_joint2 = (0.0001,0.9)
        # self.stopCFM_joint3 = (0.0001,0.9)
        # self.stopCFM_joint4 = (0.0001,0.9)
        # self.stopCFM_joint5 = (0.0001,0.9)
        # self.stopCFM_joint6 = (0.0001,0.9)
        # self.bounce_joint1 = (0.0001,0.9)
        # self.bounce_joint2 = (0.0001,0.9)
        # self.bounce_joint3 = (0.0001,0.9)
        # self.bounce_joint4 = (0.0001,0.9)
        # self.bounce_joint5 = (0.0001,0.9)
        # self.bounce_joint6 = (0.0001,0.9)
            #Material Properties
        self.friction_gripper = (0.0001, 1.25)
        self.friction_floor = (0.0001, 1.25)
        self.friction_wood = (0.0001, 1.25)
        self.friction_plastic = (0.0001, 1.25)
        self.rollingFriction_gripper = (0.0001, 1.25)
        self.rollingFriction_floor = (0.0001, 1.25)
        self.rollingFriction_wood = (0.0001, 1.25)
        self.rollingFriction_plastic = (0.0001, 1.25)
        self.slidingFriction_gripper = (0.0001, 1.25)
        self.slidingFriction_floor = (0.0001, 1.25)
        self.slidingFriction_wood = (0.0001, 1.25)
        self.slidingFriction_plastic = (0.0001, 1.25)
        self.restitution_gripper = (0.0001, 0.90)
        self.restitution_floor = (0.0001, 0.90)
        self.restitution_wood = (0.0001, 0.90)
        self.restitution_plastic = (0.0001, 0.90)
        self.linearDamping_gripper = (0.0001, 0.90)
        self.linearDamping_floor = (0.0001, 0.90)
        self.linearDamping_wood = (0.0001, 0.90)
        self.linearDamping_plastic = (0.0001, 0.90)
        self.angularDamping_gripper = (0.0001, 0.90)
        self.angularDamping_floor = (0.0001, 0.90)
        self.angularDamping_wood = (0.0001, 0.90)
        self.angularDamping_plastic = (0.0001, 0.90)
        # self.contactStiffness_gripper = (0, 1)
        # self.contactStiffness_floor = (0, 1)
        # self.contactStiffness_wood = (0, 1)
        # self.contactStiffness_plastic = (0, 1)
        # self.contactDamping_gripper = (0, 1)
        # self.contactDamping_floor = (0, 1)
        # self.contactDamping_wood = (0, 1)
        # self.contactDamping_plastic = (0, 1)
        # self.linearDrag_gripper = (0, 1)
        # self.linearDrag_floor = (0, 1)
        # self.linearDrag_wood = (0, 1)
        # self.linearDrag_plastic = (0, 1)
        # self.angularDrag_gripper = (0, 1)
        # self.angularDrag_floor = (0, 1)
        # self.angularDrag_wood = (0, 1)
        # self.angularDrag_plastic = (0, 1)

    def setSharedParams(self,params):
        """Sets the parameters for our simulation object, allows setters and getters to be used"""
        self._timeStep = params[0]
        self._mass_link1 = params[1]
        self._mass_link2 = params[2]
        self._mass_link3 = params[3]
        self._mass_link4 = params[4]
        self._mass_link5 = params[5]
        self._mass_link6 = params[6]
        self._mass_cubeP = params[7]
        self._mass_cubeW = params[8]
        self._mass_cylinderP = params[9]
        self._mass_cylinderW = params[10]
        self._mass_coneP = params[11]
        self._mass_coneW = params[12]
        self._mass_cuboidP = params[13]
        self._mass_cuboidW = params[14]
        
        self._maxForce_joint1 = params[15]
        self._maxForce_joint2 = params[16]
        self._maxForce_joint3 = params[17]
        self._maxForce_joint4 = params[18]
        self._maxForce_joint5 = params[19]
        self._maxForce_joint6 = params[20]
        self._maxVelocity_joint1 = params[21]
        self._maxVelocity_joint2 = params[22]
        self._maxVelocity_joint3 = params[23]
        self._maxVelocity_joint4 = params[24]
        self._maxVelocity_joint5 = params[25]
        self._maxVelocity_joint6 = params[26]
        
        self._friction_gripper = params[27]
        self._friction_floor = params[28]
        self._friction_wood = params[29]
        self._friction_plastic = params[30]
        

    def setIndividualParams(self,params):
        """Sets the parameters for our simulation object, allows setters and getters to be used"""
        self._timeStep = params[0]
        self._mass_link1 = params[1]
        self._mass_link2 = params[2]
        self._mass_link3 = params[3]
        self._mass_link4 = params[4]
        self._mass_link5 = params[5]
        self._mass_link6 = params[6]
        self._mass_cubeP = params[7]
        self._mass_cubeW = params[8]
        self._mass_cylinderP = params[9]
        self._mass_cylinderW = params[10]
        self._mass_coneP = params[11]
        self._mass_coneW = params[12]
        self._mass_cuboidP = params[13]
        self._mass_cuboidW = params[14]
        
        self._maxForce_joint1 = params[15]
        self._maxForce_joint2 = params[16]
        self._maxForce_joint3 = params[17]
        self._maxForce_joint4 = params[18]
        self._maxForce_joint5 = params[19]
        self._maxForce_joint6 = params[20]
        self._maxVelocity_joint1 = params[21]
        self._maxVelocity_joint2 = params[22]
        self._maxVelocity_joint3 = params[23]
        self._maxVelocity_joint4 = params[24]
        self._maxVelocity_joint5 = params[25]
        self._maxVelocity_joint6 = params[26]
        
        self._friction_gripper = params[27]
        self._friction_floor = params[28]
        self._friction_wood = params[29]
        self._friction_plastic = params[30]


        self._jointDamping_joint1 = params[31]
        self._jointDamping_joint2 = params[32]
        self._jointDamping_joint3 = params[33]
        self._jointDamping_joint4 = params[34]
        self._jointDamping_joint5 = params[35]
        self._jointDamping_joint6 = params[36]

        self._rollingFriction_gripper = params[37]
        self._rollingFriction_floor = params[38]
        self._rollingFriction_wood = params[39]
        self._rollingFriction_plastic = params[40]
        self._slidingFriction_gripper = params[41]
        self._slidingFriction_floor = params[42]
        self._slidingFriction_wood = params[43]
        self._slidingFriction_plastic = params[44]
        self._restitution_gripper = params[45]
        self._restitution_floor = params[46]
        self._restitution_wood = params[47]
        self._restitution_plastic = params[48]
        self._linearDamping_gripper = params[49]
        self._linearDamping_floor = params[50]
        self._linearDamping_wood = params[51]
        self._linearDamping_plastic = params[52]
        self._angularDamping_gripper = params[53]
        self._angularDamping_floor = params[54]
        self._angularDamping_wood = params[55]
        self._angularDamping_plastic = params[56]

    def get_timeStep(self,): return self._timeStep
    def get_mass_link1(self,): return self._mass_link1
    def get_mass_link2(self,): return self._mass_link2
    def get_mass_link3(self,): return self._mass_link3
    def get_mass_link4(self,): return self._mass_link4
    def get_mass_link5(self,): return self._mass_link5  
    def get_mass_link6(self,): return self._mass_link6
    def get_mass_cubeP(self,): return self._mass_cubeP
    def get_mass_cubeW(self,): return self._mass_cubeW
    def get_mass_cylinderP(self,): return self._mass_cylinderP
    def get_mass_cylinderW (self,): return self._mass_cylinderW
    def get_mass_coneP (self,): return self._mass_coneP
    def get_mass_coneW (self,): return self._mass_coneW
    def get_mass_cuboidP (self,): return self._mass_cuboidP
    def get_mass_cuboidW (self,): return self._mass_cuboidW
    # def get_inertia_link1_x(self,): return self._inertia_link1_x
    # def get_inertia_link1_y(self,): return self._inertia_link1_y
    # def get_inertia_link1_z(self,): return self._inertia_link1_z
    # def get_inertia_link2_x(self,): return self._inertia_link2_x
    # def get_inertia_link2_y(self,): return self._inertia_link2_y
    # def get_inertia_link2_z(self,): return self._inertia_link2_z
    # def get_inertia_link3_x(self,): return self._inertia_link3_x
    # def get_inertia_link3_y(self,): return self._inertia_link3_y
    # def get_inertia_link3_z(self,): return self._inertia_link3_z
    # def get_inertia_link4_x(self,): return self._inertia_link4_x
    # def get_inertia_link4_y(self,): return self._inertia_link4_y
    # def get_inertia_link4_z(self,): return self._inertia_link4_z
    # def get_inertia_link5_x(self,): return self._inertia_link5_x
    # def get_inertia_link5_y(self,): return self._inertia_link5_y
    # def get_inertia_link5_z(self,): return self._inertia_link5_z
    # def get_inertia_link6_x(self,): return self._inertia_link6_x
    # def get_inertia_link6_y(self,): return self._inertia_link6_y
    # def get_inertia_link6_z(self,): return self._inertia_link6_z

    def get_maxForce_joint1(self,): return self._maxForce_joint1
    def get_maxForce_joint2(self,): return self._maxForce_joint2
    def get_maxForce_joint3(self,): return self._maxForce_joint3
    def get_maxForce_joint4(self,): return self._maxForce_joint4
    def get_maxForce_joint5(self,): return self._maxForce_joint5
    def get_maxForce_joint6(self,): return self._maxForce_joint6
    def get_maxVelocity_joint1(self,): return self._maxVelocity_joint1
    def get_maxVelocity_joint2(self,): return self._maxVelocity_joint2
    def get_maxVelocity_joint3(self,): return self._maxVelocity_joint3
    def get_maxVelocity_joint4(self,): return self._maxVelocity_joint4
    def get_maxVelocity_joint5(self,): return self._maxVelocity_joint5
    def get_maxVelocity_joint6(self,): return self._maxVelocity_joint6
    def get_jointDamping_joint1(self,): return self._jointDamping_joint1
    def get_jointDamping_joint2(self,): return self._jointDamping_joint2
    def get_jointDamping_joint3(self,): return self._jointDamping_joint3
    def get_jointDamping_joint4(self,): return self._jointDamping_joint4
    def get_jointDamping_joint5(self,): return self._jointDamping_joint5
    def get_jointDamping_joint6(self,): return self._jointDamping_joint6
    # def get_stopERP_joint1(self,): return self._stopERP_joint1
    # def get_stopERP_joint2(self,): return self._stopERP_joint2
    # def get_stopERP_joint3(self,): return self._stopERP_joint3
    # def get_stopERP_joint4(self,): return self._stopERP_joint4
    # def get_stopERP_joint5(self,): return self._stopERP_joint5
    # def get_stopERP_joint6(self,): return self._stopERP_joint6
    # def get_normalCFM_joint1(self,): return self._normalCFM_joint1
    # def get_normalCFM_joint2(self,): return self._normalCFM_joint2
    # def get_normalCFM_joint3(self,): return self._normalCFM_joint3
    # def get_normalCFM_joint4(self,): return self._normalCFM_joint4
    # def get_normalCFM_joint5(self,): return self._normalCFM_joint5
    # def get_normalCFM_joint6(self,): return self._normalCFM_joint6
    # def get_stopCFM_joint1(self,): return self._stopCFM_joint1
    # def get_stopCFM_joint2(self,): return self._stopCFM_joint2
    # def get_stopCFM_joint3(self,): return self._stopCFM_joint3
    # def get_stopCFM_joint4(self,): return self._stopCFM_joint4
    # def get_stopCFM_joint5(self,): return self._stopCFM_joint5
    # def get_stopCFM_joint6(self,): return self._stopCFM_joint6
    # def get_bounce_joint1(self,): return self._bounce_joint1
    # def get_bounce_joint2(self,): return self._bounce_joint2
    # def get_bounce_joint3(self,): return self._bounce_joint3
    # def get_bounce_joint4(self,): return self._bounce_joint4
    # def get_bounce_joint5(self,): return self._bounce_joint5
    # def get_bounce_joint6(self,): return self._bounce_joint6

    def get_friction_gripper(self,): return self._friction_gripper
    def get_friction_floor(self,): return self._friction_floor
    def get_friction_wood(self,): return self._friction_wood
    def get_friction_plastic(self,): return self._friction_plastic
    def get_rollingFriction_gripper(self,): return self._rollingFriction_gripper
    def get_rollingFriction_floor(self,): return self._rollingFriction_floor
    def get_rollingFriction_wood(self,): return self._rollingFriction_wood
    def get_rollingFriction_plastic(self,): return self._rollingFriction_plastic
    def get_slidingFriction_gripper(self,): return self._slidingFriction_gripper
    def get_slidingFriction_floor(self,): return self._slidingFriction_floor
    def get_slidingFriction_wood(self,): return self._slidingFriction_wood
    def get_slidingFriction_plastic(self,): return self._slidingFriction_plastic
    def get_restitution_gripper(self,): return self._restitution_gripper
    def get_restitution_floor(self,): return self._restitution_floor
    def get_restitution_wood(self,): return self._restitution_wood
    def get_restitution_plastic(self,): return self._restitution_plastic
    def get_linearDamping_gripper(self,): return self._linearDamping_gripper
    def get_linearDamping_floor(self,): return self._linearDamping_floor
    def get_linearDamping_wood(self,): return self._linearDamping_wood
    def get_linearDamping_plastic(self,): return self._linearDamping_plastic
    def get_angularDamping_gripper(self,): return self._angularDamping_gripper
    def get_angularDamping_floor(self,): return self._angularDamping_floor
    def get_angularDamping_wood(self,): return self._angularDamping_wood
    def get_angularDamping_plastic(self,): return self._angularDamping_plastic
    # def get_contactStiffness_gripper(self,): return self._contactStiffness_gripper
    # def get_contactStiffness_floor(self,): return self._contactStiffness_floor
    # def get_contactStiffness_wood(self,): return self._contactStiffness_wood
    # def get_contactStiffness_plastic(self,): return self._contactStiffness_plastic
    # def get_contactDamping_gripper(self,): return self._contactDamping_gripper
    # def get_contactDamping_floor(self,): return self._contactDamping_floor
    # def get_contactDamping_wood(self,): return self._contactDamping_wood
    # def get_contactDamping_plastic(self,): return self._contactDamping_plastic
    # def get_linearDrag_gripper(self,): return self._linearDrag_gripper
    # def get_linearDrag_floor(self,): return self._linearDrag_floor
    # def get_linearDrag_wood(self,): return self._linearDrag_wood
    # def get_linearDrag_plastic(self,): return self._linearDrag_plastic
    # def get_angularDrag_gripper(self,): return self._angularDrag_gripper
    # def get_angularDrag_floor(self,): return self._angularDrag_floor
    # def get_angularDrag_wood(self,): return self._angularDrag_wood
    # def get_angularDrag_plastic(self,): return self._angularDrag_plastic

    def sharedParamList(self,):
        return [self.timeStep,
        self.mass_link1,
        self.mass_link2,
        self.mass_link3,
        self.mass_link4,
        self.mass_link5,
        self.mass_link6,
        self.mass_cubeP,
        self.mass_cubeW,
        self.mass_cylinderP,
        self.mass_cylinderW,
        self.mass_coneP,
        self.mass_coneW,
        self.mass_cuboidP,
        self.mass_cuboidW,
        self.maxForce_joint1,
        self.maxForce_joint2,
        self.maxForce_joint3,
        self.maxForce_joint4,
        self.maxForce_joint5,
        self.maxForce_joint6,
        self.maxVelocity_joint1,
        self.maxVelocity_joint2,
        self.maxVelocity_joint3,
        self.maxVelocity_joint4,
        self.maxVelocity_joint5,
        self.maxVelocity_joint6,
        self.friction_gripper,
        self.friction_floor,
        self.friction_wood,
        self.friction_plastic]

    def individualParamList(self,):
        return [self.timeStep,
        self.mass_link1,
        self.mass_link2,
        self.mass_link3,
        self.mass_link4,
        self.mass_link5,
        self.mass_link6,
        self.mass_cubeP,
        self.mass_cubeW,
        self.mass_cylinderP,
        self.mass_cylinderW,
        self.mass_coneP,
        self.mass_coneW,
        self.mass_cuboidP,
        self.mass_cuboidW,
        self.maxForce_joint1,
        self.maxForce_joint2,
        self.maxForce_joint3,
        self.maxForce_joint4,
        self.maxForce_joint5,
        self.maxForce_joint6,
        self.maxVelocity_joint1,
        self.maxVelocity_joint2,
        self.maxVelocity_joint3,
        self.maxVelocity_joint4,
        self.maxVelocity_joint5,
        self.maxVelocity_joint6,
        self.friction_gripper,
        self.friction_floor,
        self.friction_wood,
        self.friction_plastic,
        self.jointDamping_joint1,
        self.jointDamping_joint2,
        self.jointDamping_joint3,
        self.jointDamping_joint4,
        self.jointDamping_joint5,
        self.jointDamping_joint6,  
        self.rollingFriction_gripper,
        self.rollingFriction_floor,
        self.rollingFriction_wood,
        self.rollingFriction_plastic,
        self.slidingFriction_gripper,
        self.slidingFriction_floor,
        self.slidingFriction_wood,
        self.slidingFriction_plastic,
        self.restitution_gripper,
        self.restitution_floor,
        self.restitution_wood,
        self.restitution_plastic,
        self.linearDamping_gripper,
        self.linearDamping_floor,
        self.linearDamping_wood,
        self.linearDamping_plastic,
        self.angularDamping_gripper,
        self.angularDamping_floor,
        self.angularDamping_wood,
        self.angularDamping_plastic]
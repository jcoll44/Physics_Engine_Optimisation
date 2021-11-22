# -*- coding: utf-8 -*-
"""
Created on Mon Sep 24 12:01:49 2018

@author: jack
"""

from kinova_pb import *
from kinova_vr import *
from Rotations import *
from fitness import *
from logger import *
from params import *

from scipy.optimize import differential_evolution
import numpy as np
import time
import sys
import multiprocessing
# from random import randint 
# from mpi4py import MPI
# from mpi4py.futures import MPIPoolExecutor

optimiseRuns = 250
population = 1
repeats = 20

class OPTIMISE(object):
    def __init__(self, simulator, repeat, experiment, sharedParams):
        self._physics_engine = simulator
        self._repeat = repeat
        self._optimiseRuns = optimiseRuns
        self._experiment = experiment
        self._sharedParams = sharedParams

        self._logging = LOGGER(self._physics_engine, self._experiment,self._repeat)
        self._params = PARAMS(self._sharedParams)
        self._fitness = FITNESS(self._physics_engine, self._experiment,self._repeat)

        self._generationNum = 0

        self._logging.createDirectory()

    def setGeneration(self,data, convergence):
        self._generationNum += 1
        

    def evolve(self,):
        """This function completed the tuning and saving of results, gp_minimize is responsible for tuning
        the parameters after each run of the 10 types of simulations"""
        # executor = MPIPoolExecutor(max_workers=5)

        np.random.seed(self._repeat)
        if self._sharedParams == True:
            result = differential_evolution(self.allTasks,self._params.sharedParamList(), popsize = 1,workers = 1, callback=self.setGeneration)
        elif self._sharedParams == False:
            result = differential_evolution(self.allTasks,self._params.individualParamList(), popsize = 1,workers = 1, callback=self.setGeneration)
        self._logging.saveParamsEnd(result)
        self._logging.save_convergence_plot(result)
        print(result)
 

    def allTasks(self, params):
        """Generates all task parameters to match physics engine
        This function is called by gp_minimize, all 10 simulations are run, their fitness returned to gp_minimise so that gp_minimise
        can tune parameters before calling allTasks again. This is called as many times as optimiseRuns"""
        if self._sharedParams == True:
            self._params.setSharedParams(params)        
        elif self._sharedParams == False:
            self._params.setIndividualParams(params) 
        if self._experiment <= 10:
            if self._physics_engine == 'PyBullet':
                simulate = PYBULLET(self._experiment,self._generationNum, self._sharedParams, self._params, self._fitness)
                fitness = simulate.run_pybullet()
            elif self._physics_engine == 'Bullet283':
                simulate = VREP(self._experiment, self._generationNum, self._sharedParams, self._params, self._fitness)
                fitness = simulate.Bullet283()
            elif self._physics_engine == 'Bullet278':
                simulate = VREP(self._experiment, self._generationNum, self._sharedParams, self._params, self._fitness)
                fitness = simulate.Bullet278()
            elif self._physics_engine == 'ODE':
                simulate = VREP(self._experiment, self._generationNum, self._sharedParams, self._params, self._fitness)
                fitness = simulate.ODE()
            elif self._physics_engine == 'Vortex':
                simulate = VREP(self._experiment, self._generationNum, self._sharedParams, self._params, self._fitness)
                fitness = simulate.Vortex()
            elif self._physics_engine == 'Newton':
                simulate = VREP(self._experiment, self._generationNum, self._sharedParams, self._params, self._fitness)
                fitness = simulate.Newton()
        else:
            fitness = 0
            for experimentnum in range(1,11):
                if self._physics_engine == 'PyBullet':
                    simulate = PYBULLET(experimentnum,self._generationNum, self._sharedParams, self._params, self._fitness)
                    fitness += simulate.run_pybullet()
                elif self._physics_engine == 'Bullet283':
                    simulate = VREP(experimentnum, self._generationNum, self._sharedParams, self._params, self._fitness)
                    fitness += simulate.Bullet283()
                elif self._physics_engine == 'Bullet278':
                    simulate = VREP(experimentnum, self._generationNum, self._sharedParams, self._params, self._fitness)
                    fitness += simulate.Bullet278()
                elif self._physics_engine == 'ODE':
                    simulate = VREP(experimentnum, self._generationNum, self._sharedParams, self._params, self._fitness)
                    fitness += simulate.ODE()
                elif self._physics_engine == 'Vortex':
                    simulate = VREP(experimentnum, self._generationNum, self._sharedParams, self._params, self._fitness)
                    fitness += simulate.Vortex()
                elif self._physics_engine == 'Newton':
                    simulate = VREP(experimentnum, self._generationNum, self._sharedParams, self._params, self._fitness)
                    fitness += simulate.Newton()
        self._logging.saveParams(params, fitness, self._generationNum)
        return fitness

if __name__ == '__main__':
    arguments = sys.argv[1:]

    physics_engine = str(arguments[0])
    print(physics_engine)
    experiment = int(arguments[1])  # This is now 1-6. Where 1 = 1&2, 2 = 3&4, ..., 6 = 11
    print(experiment)
    sharedParams_int = int(arguments[2])
    if sharedParams_int == 0:
        sharedParams = False
    elif sharedParams_int == 1:
        sharedParams = True
    print(sharedParams)

    tic = time.time()
    jobs = []
    optimise = []
    for repeat in range(int(repeats/2)):
        optimise.append(OPTIMISE(physics_engine, repeat, (experiment*2)-1, sharedParams))
        p = multiprocessing.Process(target=optimise[-1].evolve)
        jobs.append(p)
        p.start()
    if experiment != 6:
        for repeat in range(int(repeats/2),repeats):
            optimise.append(OPTIMISE(physics_engine, (repeat-int(repeats/2)), experiment*2, sharedParams))
            p = multiprocessing.Process(target=optimise[-1].evolve)
            jobs.append(p)
            p.start()
    for j in jobs:
        j.join()
    toc = time.time()
    print(toc - tic, " seconds")

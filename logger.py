# -*- coding: utf-8 -*-
"""
Created on Mon Aug 26 18:04:20 2019

@author: jack
"""

import csv, os
from scipy.optimize import OptimizeResult
from skopt.plots import plot_convergence
from skopt.plots import plot_objective
from skopt.plots import plot_evaluations
import matplotlib.pyplot as plt
from sklearn.externals.joblib import dump
from matplotlib.pyplot import cm
import numpy as np

class LOGGER(object):
    """Logger object to save stats, parameters and plots"""
    def __init__(self, physics_engine, experiment, repeat):

        self._physics_engine = physics_engine
        self._experiment = experiment

        self._repeat = repeat
        self._task = 0


    #Save simulation Data
    def saveStats(self, data, optimisationRun, task):
        """Saves raw simulation statistics for each gp_minimise call, saves for each physics engine, optimisation and repeat"""
        self._optimisationRun = optimisationRun
        self._task = task
        my_path = os.path.abspath(os.path.dirname(__file__))
        if self._optimisationRun < 10:
            fileStringComp = 'Results/%s/experiment%d/Repeat_%d/00%d/task%d.csv'%(self._physics_engine, self._experiment, self._repeat, self._optimisationRun, self._task)
        elif self._optimisationRun < 100:
            fileStringComp = 'Results/%s/experiment%d/Repeat_%d/0%d/task%d.csv'%(self._physics_engine, self._experiment, self._repeat, self._optimisationRun, self._task)
        else:
            fileStringComp = 'Results/%s/experiment%d/Repeat_%d/%d/task%d.csv'%(self._physics_engine, self._experiment, self._repeat, self._optimisationRun, self._task)
        path = os.path.join(my_path, fileStringComp)
        
        os.makedirs(os.path.dirname(path), exist_ok=True)

        with open(path, 'w+', newline='') as csvfile:
            writer = csv.writer(csvfile, delimiter=',', quoting=csv.QUOTE_MINIMAL)
            for xyz in data:
                writer.writerow(xyz)

    #Save Optimised Parameters
    def saveParams(self,data,fitness, generationNum):
        """Saves simulation parameters used for this simulation run gp_minimise"""
        self._generationNum = generationNum
        my_path = os.path.abspath(os.path.dirname(__file__))
        if self._generationNum < 10:
            fileStringComp = 'Results/%s/experiment%d/Repeat_%d/Parameters_00%d.csv'%(self._physics_engine, self._experiment, self._repeat, self._generationNum)
        elif self._generationNum < 100:
            fileStringComp = 'Results/%s/experiment%d/Repeat_%d/Parameters_0%d.csv'%(self._physics_engine, self._experiment, self._repeat, self._generationNum)
        else:
            fileStringComp = 'Results/%s/experiment%d/Repeat_%d/Parameters_%d.csv'%(self._physics_engine, self._experiment, self._repeat, self._generationNum)
        path = os.path.join(my_path, fileStringComp)

        # os.makedirs(os.path.dirname(path), exist_ok=True)

        with open(path, 'a+', newline='') as csvfile:
            writer = csv.writer(csvfile, delimiter=',', quoting=csv.QUOTE_MINIMAL)
            writer.writerow(np.insert(data,0,fitness))

    #Save Optimised Parameters
    def saveParamsEnd(self,data):
        """Save final/tuned simulation parameters optimised for this simulation run gp_minimise"""
        my_path = os.path.abspath(os.path.dirname(__file__))
        file = 'Results/%s/experiment%d/Repeat_%d/Parameters_end.csv'%(self._physics_engine, self._experiment, self._repeat)
        path = os.path.join(my_path, file)

        # os.makedirs(os.path.dirname(path), exist_ok=True)

        with open(path, 'a+', newline='') as csvfile:
            writer = csv.writer(csvfile, delimiter=',', quoting=csv.QUOTE_MINIMAL)
            writer.writerow(data.x)

        self.dump_data(data)

    def createDirectory(self,):
        my_path = os.path.abspath(os.path.dirname(__file__))
        file = 'Results/%s/experiment%d/Repeat_%d/'%(self._physics_engine, self._experiment, self._repeat)
        path = os.path.join(my_path, file)
        os.makedirs(os.path.dirname(path), exist_ok=True)

    def dump_data(self, data):
        my_path = os.path.abspath(os.path.dirname(__file__))
        file = 'Results/%s/experiment%d/Repeat_%d/result.pkl'%(self._physics_engine, self._experiment, self._repeat)
        path = os.path.join(my_path, file)
        dump(data, path)

    def save_convergence_plot(self, results_obj):
        """Save Gausian Optimisation convergance plot see https://scikit-optimize.github.io/#skopt.gp_minimize """
        my_path = os.path.abspath(os.path.dirname(__file__))

        plt.figure(1)

        plt.suptitle("Convergence plot")
        plt.xlabel("Number of calls $n$")
        plt.ylabel(r"$\min f(x)$ after $n$ calls")
        plt.grid()

        colors = cm.viridis(np.linspace(0.25, 1.0, 3))

        # for results, color in zip(results_obj, colors):
        #     name = None

        n_calls = results_obj.nit
        mean = np.empty([n_calls+1])
        best = np.empty([n_calls+1])
        worst = np.empty([n_calls+1])
        for gen in range(0,n_calls+1):
            if gen < 10:
                fileStringComp = 'Results/%s/experiment%d/Repeat_%d/Parameters_00%d.csv'%(self._physics_engine, self._experiment, self._repeat, gen)
            elif gen < 100:
                fileStringComp = 'Results/%s/experiment%d/Repeat_%d/Parameters_0%d.csv'%(self._physics_engine, self._experiment, self._repeat, gen)
            else:
                fileStringComp = 'Results/%s/experiment%d/Repeat_%d/Parameters_%d.csv'%(self._physics_engine, self._experiment, self._repeat, gen)
            path = os.path.join(my_path, fileStringComp)
            temp_data = np.genfromtxt(path,delimiter=',',usecols=0)
            mean[gen] = np.mean(temp_data)
            best[gen] = np.amin(temp_data)
            worst[gen] = np.amax(temp_data)
        best_mins = [np.min(best[:i]) 
                    for i in range(1, n_calls+2)]
        plt.plot(range(0, n_calls+1), best_mins, c=colors[0],
                marker=".", markersize=12, lw=2, label="Best")
        plt.plot(range(0, n_calls+1), worst, c=colors[1],
                marker=".", markersize=12, lw=2, label="Worst")
        plt.plot(range(0, n_calls+1), mean, c=colors[2],
                marker=".", markersize=12, lw=2, label="Mean")
        
        file = 'Results/%s/experiment%d/Repeat_%d/savedfigConv.png'%(self._physics_engine, self._experiment, self._repeat)
        path = os.path.join(my_path, file)
        plt.savefig(path, dpi=None, facecolor='w', edgecolor='w',
            orientation='portrait', papertype=None, format=None,
            transparent=False, bbox_inches=None, pad_inches=0.1,
            frameon=None, metadata=None)


    def save_gp_plots(self, results_obj):
        """Save Gausian Optimisation objective and evaluations plots see https://scikit-optimize.github.io/plots.m.html """
        my_path = os.path.abspath(os.path.dirname(__file__))

        #Objective
        plt.figure(1)
        file = 'Results/%s/experiment%d/Repeat_%d/savedfigObj.png'%(self._physics_engine, self._experiment, self._repeat)
        path = os.path.join(my_path, file)
        #Note, this plot takes time to load at the end. Dont Panic!
        plot_objective(results_obj)
        plt.savefig(path, dpi=None, facecolor='w', edgecolor='w',
            orientation='portrait', papertype=None, format=None,
            transparent=False, bbox_inches=None, pad_inches=0.1,
            frameon=None, metadata=None)
        #Evaluations
        plt.figure(2)
        file = 'Results/%s/experiment%d/Repeat_%d/savedfigEval.png'%(self._physics_engine, self._experiment, self._repeat)
        path = os.path.join(my_path, file)
        plot_evaluations(results_obj, bins=20, dimensions=None)
        plt.savefig(path, dpi=None, facecolor='w', edgecolor='w',
            orientation='portrait', papertype=None, format=None,
            transparent=False, bbox_inches=None, pad_inches=0.1,
            frameon=None, metadata=None)
import matplotlib.pyplot as plt
from matplotlib.pyplot import cm
import numpy as np
import os
import math


#Fitness Convergence Plots
#1 subplot per task
#1 line per physics engine
#line bouneded by stdev

#Location of Results
folder = '/home/col549/Desktop/pearcey_shared/Results'

#Global Variables
experiments = 11
repeats = 10
physics_engine = ["PyBullet", "Bullet278", "Bullet283", "ODE", "Newton"]

# #Setup Plot
# # Creates 11 subplots
# f1, axes = plt.subplots(11, 1)
# axes[0].set_title('Task 1')
# axes[1].set_title('Task 2')
# axes[2].set_title('Task 3')
# axes[3].set_title('Task 4')
# axes[4].set_title('Task 5')
# axes[5].set_title('Task 6')
# axes[6].set_title('Task 7')
# axes[7].set_title('Task 8')
# axes[8].set_title('Task 9')
# axes[9].set_title('Task 10')
# axes[10].set_title('Task 11')
# colours = cm.viridis(np.linspace(0.25, 1.0, len(physics_engine)))


# #Read in data into array
# for experiment in range(1,experiments+1):
#     engine_list = []
#     for engine in range(len(physics_engine)):
#         repeat_list = []
#         for repeat in range(repeats):
#             end = False
#             param = 0
#             temp_list = []
#             while end == False:
#                 try:
#                     if param < 10:
#                         fileStringComp = '/%s/experiment%d/Repeat_%d/Parameters_00%d.csv'%(physics_engine[engine], experiment, repeat, param)
#                     elif param < 100:
#                         fileStringComp = '/%s/experiment%d/Repeat_%d/Parameters_0%d.csv'%(physics_engine[engine], experiment, repeat, param)
#                     else:
#                         fileStringComp = '/%s/experiment%d/Repeat_%d/Parameters_%d.csv'%(physics_engine[engine], experiment, repeat, param)

#                     path = folder+fileStringComp
#                     temp_data = np.genfromtxt(path,delimiter=',',usecols=0)
#                     print(path)
#                     print(temp_data)
#                     temp_list.append(np.amin(temp_data))
#                     param += 1
#                 except:
#                     end = True
#                     repeat_list.append(temp_list)

#         longest_list = 0
#         for List in repeat_list:
#             if len(List) > longest_list:
#                 longest_list = len(List)
#         for List in repeat_list:
#             if len(List) != longest_list:
#                 for x in range(len(List),longest_list):
#                     List.append(List[-1])
#         print(repeat_list)
#         repeat_list = np.array(repeat_list)
#         print(repeat_list)
#         repeat_list = np.mean(repeat_list,axis=0)
#         print(repeat_list)
#         for x in range(len(repeat_list)-1):
#             if repeat_list[x] < repeat_list[x+1]:
#                 repeat_list[x+1] = repeat_list[x]
#         if experiment==1:
#             axes[experiment-1].plot(range(0,repeat_list.shape[0]),repeat_list, c=colours[engine], label=physics_engine[engine])
#         else:
#             axes[experiment-1].plot(range(0,repeat_list.shape[0]),repeat_list, c=colours[engine])
# f1.tight_layout()
# f1.legend(loc='center right')
# f1.savefig(folder, dpi=None, facecolor='w', edgecolor='w',
#             orientation='portrait', papertype=None, format=None,
#             transparent=False, bbox_inches=None, pad_inches=0.1,
#             frameon=None, metadata=None)
# f1.show()


# #Boxplot ALL VARIABLES
# #1 subplot per task
# #1 boxplot per variable/parameter

# #Global Variables
# shared_params = 31
# columns = len(physics_engine)*shared_params

# #Setup Plot
# # Creates 11 subplots
# f2, axes = plt.subplots(11, 1)
# axes[0].set_title('Task 1')
# axes[1].set_title('Task 2')
# axes[2].set_title('Task 3')
# axes[3].set_title('Task 4')
# axes[4].set_title('Task 5')
# axes[5].set_title('Task 6')
# axes[6].set_title('Task 7')
# axes[7].set_title('Task 8')
# axes[8].set_title('Task 9')
# axes[9].set_title('Task 10')
# axes[10].set_title('Task 11')
# colours = cm.viridis(np.linspace(0.25, 1.0, len(physics_engine)))


# #Read data into array
# for experiment in range(1,experiments+1):
#     results_array = np.empty((10,columns))   
#     for col in range(columns):
#         phy = col % len(physics_engine)
#         param = col % shared_params
#         for repeat in range(repeats):
#             fileStringComp = '/%s/experiment%d/Repeat_%d/Parameters_end.csv'%(physics_engine[phy], experiment, repeat)
#             try:
#                 path = folder+fileStringComp
#                 temp_data = np.genfromtxt(path,delimiter=',', usecols=param, max_rows=1)
#                 print(path)
#                 print(temp_data)
#                 results_array[repeat,col]=temp_data
#             except:
#                 end = False
#                 parameter = 1000
#                 while end == False:
#                     print(parameter)
#                     if parameter < 10:
#                         fileStringComp = '/%s/experiment%d/Repeat_%d/Parameters_00%d.csv'%(physics_engine[phy], experiment, repeat, parameter)
#                     elif parameter < 100:
#                         fileStringComp = '/%s/experiment%d/Repeat_%d/Parameters_0%d.csv'%(physics_engine[phy], experiment, repeat, parameter)
#                     else:
#                         fileStringComp = '/%s/experiment%d/Repeat_%d/Parameters_%d.csv'%(physics_engine[phy], experiment, repeat, parameter)
#                     path = folder+fileStringComp
#                     print(path)
#                     try:
#                         temp_data = np.genfromtxt(path,delimiter=',') #+1 because there is a fitness in the first col
#                         end = True
#                         temp_data = np.array(temp_data)
#                         temp_min = temp_data
#                         min_fit = np.where(temp_min[:,0] == np.amin(temp_min[:,0]))
#                         temp_data = temp_data[min_fit[0][0],param+1]
#                         results_array[repeat,col]=temp_data
#                     except:
#                         parameter-=1

#     print(results_array)  
#     axes[experiment-1].boxplot(results_array)

# # f2.tight_layout()
# # f2.legend(loc='center right')
# f2.savefig(folder+"boxplot.png", dpi=None, facecolor='w', edgecolor='w',
#             orientation='portrait', papertype=None, format=None,
#             transparent=False, bbox_inches=None, pad_inches=0.1,
#             frameon=None, metadata=None)
# f2.show()


#Boxplot single VARIABLES
#1 subplot per task
#1 boxplot per variable/parameter

#Global Variables
shared_params = 31
columns = len(physics_engine)*shared_params


#Read data into array
   
for col in range(shared_params):
    #Setup Plot
    # Creates 11 subplots
    f2, axes = plt.subplots(11, 1)
    axes[0].set_title('Task 1')
    axes[1].set_title('Task 2')
    axes[2].set_title('Task 3')
    axes[3].set_title('Task 4')
    axes[4].set_title('Task 5')
    axes[5].set_title('Task 6')
    axes[6].set_title('Task 7')
    axes[7].set_title('Task 8')
    axes[8].set_title('Task 9')
    axes[9].set_title('Task 10')
    axes[10].set_title('Task 11')
    colours = cm.viridis(np.linspace(0.25, 1.0, len(physics_engine)))
    for experiment in range(1,experiments+1):
        results_array = np.empty((10,len(physics_engine)))
        for engine in range(len(physics_engine)):
            for repeat in range(repeats):
                fileStringComp = '/%s/experiment%d/Repeat_%d/Parameters_end.csv'%(physics_engine[engine], experiment, repeat)
                path = folder+fileStringComp
                if os.path.isfile(path) == True:
                    print(path)
                    temp_data = np.genfromtxt(path,delimiter=',', usecols=col, max_rows=1)
                    results_array[repeat,engine]=temp_data
                else:
                    end = False
                    parameter = 1000
                    while end == False:
                        if parameter < 10:
                            fileStringComp = '/%s/experiment%d/Repeat_%d/Parameters_00%d.csv'%(physics_engine[engine], experiment, repeat, parameter)
                        elif parameter < 100:
                            fileStringComp = '/%s/experiment%d/Repeat_%d/Parameters_0%d.csv'%(physics_engine[engine], experiment, repeat, parameter)
                        else:
                            fileStringComp = '/%s/experiment%d/Repeat_%d/Parameters_%d.csv'%(physics_engine[engine], experiment, repeat, parameter)
                        path = folder+fileStringComp

                        if os.path.isfile(path) == True:
                            temp_data = np.genfromtxt(path,delimiter=',') #+1 because there is a fitness in the first col
                            end = True
                            print(path)
                            temp_data = np.array(temp_data)
                            if temp_data.ndim == 1:
                                results_array[repeat,engine]=temp_data[col+1]
                            else:
                                temp_min = temp_data
                                min_fit = np.where(temp_min[:,0] == np.amin(temp_min[:,0]))
                                temp_data = temp_data[min_fit[0][0],col+1]
                                results_array[repeat,engine]=temp_data
                        else:
                            parameter-=1

        print(results_array)  
        axes[experiment-1].boxplot(results_array)
    # f2.tight_layout()
    # f2.legend(loc='center right')
    f2.savefig(folder+"boxplot"+str(col)+".png", dpi=None, facecolor='w', edgecolor='w',
                orientation='portrait', papertype="b1", format=None,
                transparent=False, bbox_inches=None, pad_inches=0.4,
                frameon=None, metadata=None)
    f2.show()


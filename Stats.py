import numpy as np
import os
import math

#Location of Results
folder = '/home/col549/Desktop/pearcey_shared/Results'
ind_folder = '/home/col549/Desktop/pearcey_individual/Results'

#Global Variables
experiments = 11
repeats = 10
physics_engine = ["PyBullet", "Bullet278", "Bullet283", "ODE", "Newton"]

## Time extracted from Slurm files

# slurm_folder = '/home/col549/Desktop/pearcey_individual/slurm/slurm-401131' #00.out
# for slurm in range(35,37):
#     time =''
#     if slurm < 10:
#         slurm_num = '0%d.out'%(slurm)
#     else:
#         slurm_num = '%d.out'%(slurm)
#     slurm_path = slurm_folder+slurm_num

#     with open(slurm_path, 'r') as f:
#         lines = f.read().splitlines()
#         head = lines[0:2]
#         line = lines[-2]
#     if line[0].isdigit()==False:
#         time = 168
#     else:
#         for char in line:
#             if char.isdigit() or char == '.':
#                 time += char
#         time = float(time)/3600
#     print(head, time)


## Mean and STD Params

# #Global Variables
# shared_params = 31
# individual_params = 57
# for engine in range(len(physics_engine)):
#     engine_list = np.empty((experiments*2,individual_params), dtype = 'object')
#     for col in range(individual_params):
#         print(col)
#         for experiment in range(1,experiments+1):
#             if col >= shared_params:
#                 break
#             #Shared
#             results_array = np.empty((10,1))
#             for repeat in range(repeats):
#                 fileStringComp = '/%s/experiment%d/Repeat_%d/Parameters_end.csv'%(physics_engine[engine], experiment, repeat)
#                 path = folder+fileStringComp
#                 if os.path.isfile(path) == True:
#                     # print(path)
#                     temp_data = np.genfromtxt(path,delimiter=',', usecols=col, max_rows=1)
#                     results_array[repeat]=temp_data
#                 else:
#                     end = False
#                     parameter = 1000
#                     while end == False:
#                         if parameter < 10:
#                             fileStringComp = '/%s/experiment%d/Repeat_%d/Parameters_00%d.csv'%(physics_engine[engine], experiment, repeat, parameter)
#                         elif parameter < 100:
#                             fileStringComp = '/%s/experiment%d/Repeat_%d/Parameters_0%d.csv'%(physics_engine[engine], experiment, repeat, parameter)
#                         else:
#                             fileStringComp = '/%s/experiment%d/Repeat_%d/Parameters_%d.csv'%(physics_engine[engine], experiment, repeat, parameter)
#                         path = folder+fileStringComp

#                         if os.path.isfile(path) == True:
#                             temp_data = np.genfromtxt(path,delimiter=',') #+1 because there is a fitness in the first col
#                             end = True
#                             # print(path)
#                             temp_data = np.array(temp_data)
#                             if temp_data.ndim == 1:
#                                 results_array[repeat]=temp_data[col+1]
#                             else:
#                                 temp_min = temp_data
#                                 min_fit = np.where(temp_min[:,0] == np.amin(temp_min[:,0]))
#                                 temp_data = temp_data[min_fit[0][0],col+1]
#                                 results_array[repeat]=temp_data
#                         else:
#                             parameter-=1
#             mean = np.mean(results_array)
#             std = np.std(results_array)
#             engine_list[experiment-1,col] = str('%f +- %f'%(mean,std))
#         for experiment in range(1,experiments+1):
#             #Individual
#             results_array = np.empty((10,1))
#             for repeat in range(repeats):
#                 fileStringComp = '/%s/experiment%d/Repeat_%d/Parameters_end.csv'%(physics_engine[engine], experiment, repeat)
#                 path = ind_folder+fileStringComp
#                 if os.path.isfile(path) == True:
#                     # print(path)
#                     temp_data = np.genfromtxt(path,delimiter=',', usecols=col, max_rows=1)
#                     results_array[repeat]=temp_data
#                 else:
#                     end = False
#                     parameter = 1000
#                     while end == False:
#                         if parameter < 10:
#                             fileStringComp = '/%s/experiment%d/Repeat_%d/Parameters_00%d.csv'%(physics_engine[engine], experiment, repeat, parameter)
#                         elif parameter < 100:
#                             fileStringComp = '/%s/experiment%d/Repeat_%d/Parameters_0%d.csv'%(physics_engine[engine], experiment, repeat, parameter)
#                         else:
#                             fileStringComp = '/%s/experiment%d/Repeat_%d/Parameters_%d.csv'%(physics_engine[engine], experiment, repeat, parameter)
#                         path = ind_folder+fileStringComp

#                         if os.path.isfile(path) == True:
#                             temp_data = np.genfromtxt(path,delimiter=',') #+1 because there is a fitness in the first col
#                             end = True
#                             # print(path)
#                             temp_data = np.array(temp_data)
#                             if temp_data.ndim == 1:
#                                 results_array[repeat]=temp_data[col+1]
#                             else:
#                                 temp_min = temp_data
#                                 min_fit = np.where(temp_min[:,0] == np.amin(temp_min[:,0]))
#                                 temp_data = temp_data[min_fit[0][0],col+1]
#                                 results_array[repeat]=temp_data
#                         else:
#                             parameter-=1
#             mean = np.mean(results_array)
#             std = np.std(results_array)
#             engine_list[experiment-1+experiments,col] = str('%f +- %f'%(mean,std))
    
#     np.savetxt((folder+"/"+str(physics_engine[engine])+"_params.csv"),engine_list, delimiter=",", fmt="%s")


##Fitness and STD Dev for all Experiments



# engine_list = np.empty((experiments*2,len(physics_engine)), dtype = 'object')
# for engine in range(len(physics_engine)):
#     for experiment in range(1,experiments+1):
#         #Shared
#         results_array = np.empty((10,1))
#         for repeat in range(repeats):
#             end = False
#             parameter = 1000
#             while end == False:
#                 if parameter < 10:
#                     fileStringComp = '/%s/experiment%d/Repeat_%d/Parameters_00%d.csv'%(physics_engine[engine], experiment, repeat, parameter)
#                 elif parameter < 100:
#                     fileStringComp = '/%s/experiment%d/Repeat_%d/Parameters_0%d.csv'%(physics_engine[engine], experiment, repeat, parameter)
#                 else:
#                     fileStringComp = '/%s/experiment%d/Repeat_%d/Parameters_%d.csv'%(physics_engine[engine], experiment, repeat, parameter)
#                 path = folder+fileStringComp

#                 if os.path.isfile(path) == True:
#                     temp_data = np.genfromtxt(path,delimiter=',') #+1 because there is a fitness in the first col
#                     end = True
#                     # print(path)
#                     temp_data = np.array(temp_data)
#                     if temp_data.ndim == 1:
#                         results_array[repeat]=temp_data[0]
#                     else:
#                         temp_min = temp_data
#                         min_fit = np.amin(temp_min[:,0])
#                         results_array[repeat]=min_fit
#                 else:
#                     parameter-=1
#         mean = np.mean(results_array)
#         std = np.std(results_array)
#         engine_list[experiment-1,engine] = str('%f +- %f'%(mean,std))
#     for experiment in range(1,experiments+1):
#         #Individual
#         results_array = np.empty((10,1))
#         for repeat in range(repeats):
#             end = False
#             parameter = 1000
#             while end == False:
#                 if parameter < 10:
#                     fileStringComp = '/%s/experiment%d/Repeat_%d/Parameters_00%d.csv'%(physics_engine[engine], experiment, repeat, parameter)
#                 elif parameter < 100:
#                     fileStringComp = '/%s/experiment%d/Repeat_%d/Parameters_0%d.csv'%(physics_engine[engine], experiment, repeat, parameter)
#                 else:
#                     fileStringComp = '/%s/experiment%d/Repeat_%d/Parameters_%d.csv'%(physics_engine[engine], experiment, repeat, parameter)
#                 path = ind_folder+fileStringComp

#                 if os.path.isfile(path) == True:
#                     temp_data = np.genfromtxt(path,delimiter=',') #+1 because there is a fitness in the first col
#                     end = True
#                     # print(path)
#                     temp_data = np.array(temp_data)
#                     if temp_data.ndim == 1:
#                         results_array[repeat]=temp_data[0]
#                     else:
#                         temp_min = temp_data
#                         min_fit = np.amin(temp_min[:,0])
#                         results_array[repeat]=min_fit
#                 else:
#                     parameter-=1
#         mean = np.mean(results_array)
#         std = np.std(results_array)
#         engine_list[experiment-1+experiments,engine] = str('%f +- %f'%(mean,std))

# np.savetxt((folder+"/"+"fitness.csv"),engine_list, delimiter=",", fmt="%s")


##Optimisations Complete Mean and STD dev

engine_list = np.empty((experiments*2,len(physics_engine)), dtype = 'object')
for engine in range(len(physics_engine)):
    for experiment in range(1,experiments+1):
        #Shared
        results_array = np.empty((10,1))
        for repeat in range(repeats):
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
                    results_array[repeat]=parameter
                    end = True
                else:
                    parameter-=1
        mean = np.mean(results_array)
        std = np.std(results_array)
        engine_list[experiment-1,engine] = str('{:.1f} +- {:.1f}'.format(mean,std))
    for experiment in range(1,experiments+1):
        #Individual
        results_array = np.empty((10,1))
        for repeat in range(repeats):
            end = False
            parameter = 1000
            while end == False:
                if parameter < 10:
                    fileStringComp = '/%s/experiment%d/Repeat_%d/Parameters_00%d.csv'%(physics_engine[engine], experiment, repeat, parameter)
                elif parameter < 100:
                    fileStringComp = '/%s/experiment%d/Repeat_%d/Parameters_0%d.csv'%(physics_engine[engine], experiment, repeat, parameter)
                else:
                    fileStringComp = '/%s/experiment%d/Repeat_%d/Parameters_%d.csv'%(physics_engine[engine], experiment, repeat, parameter)
                path = ind_folder+fileStringComp

                if os.path.isfile(path) == True:
                    results_array[repeat]=parameter
                    end = True
                else:
                    parameter-=1
        mean = np.mean(results_array)
        std = np.std(results_array)
        engine_list[experiment-1+experiments,engine] = str('{:.1f} +- {:.1f}'.format(mean,std))

np.savetxt((folder+"/"+"optIters.csv"),engine_list, delimiter=",", fmt="%s")
import matplotlib.pyplot as plt
import numpy as np
import os
from skopt import gp_minimize

from skopt import load
from main import *

#Shared/Individual
#PyBullet/Bullet283/Bullet278/ODE/Newton
#Task1-11
#Reapeats:20

#1D                 ##
#2D               ######  
#3D             ###########

results = [[[[0]*20]*11]*5]*2

paramters = ["Shared", "Individual"]
physics_engine = ["PyBullet", "Bullet278", "ODE", "Newton"]


for param in range(2):
    for engine in range(5):
        for task in range(0,11):
            for repeat in range(20):
                try:       
                    # my_path = os.path.abspath(os.path.dirname(__file__))
                    file = '/home/col549/Desktop/Results/%s/%s/experiment%d/Repeat_%d/result.pkl'%(paramters[param],physics_engine[engine],task+1,repeat)
                    print(file)
                    # path = os.path.join(my_path, file)     
                    results[param][engine][task][repeat] = load(file)
                except:
                    pass


print("All stored")
#Load Parameter Values

sharedParams = 49
columns = len(physics_engine)*sharedParams
#Task1 - 6 Bars per Parameter
#Paramters= 
# Timestep [0]
# Mass (6 Links, Gripper, Objects) [1-14]
# Joint Torque [33-38]
# Joint Velocity [39-44]
# Friction (Lateral) [75-78]
# Inertia Tensor(xx,yy,zz) [15-32]
# Paramters= [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,33,34,35,36,37,38,39,40,41,42,43,44,75,76,77,78,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32]
Paramters= [range(0,15), range(33,45), range(75,79), range(15,33)]

task1 = np.zeros([20,columns])
task2 = np.zeros([20,columns])
task3 = np.zeros([20,columns])
task4 = np.zeros([20,columns])
task5 = np.zeros([20,columns])
task6 = np.zeros([20,columns])
task7 = np.zeros([20,columns])
task8 = np.zeros([20,columns])
task9 = np.zeros([20,columns])
task10 = np.zeros([20,columns])
task11 = np.zeros([20,columns])

for col in range(columns):
    phy = col % 5
    params = col % 49
    for repeat in range(20):
        try:
            task1[repeat,col] = results[0][phy][1][repeat].x[Paramters[params]]
            task2[repeat,col] = results[0][phy][2][repeat].x[Paramters[params]]
            task3[repeat,col] = results[0][phy][3][repeat].x[Paramters[params]]
            task4[repeat,col] = results[0][phy][4][repeat].x[Paramters[params]]
            task5[repeat,col] = results[0][phy][5][repeat].x[Paramters[params]]
            task6[repeat,col] = results[0][phy][6][repeat].x[Paramters[params]]
            task7[repeat,col] = results[0][phy][7][repeat].x[Paramters[params]]
            task8[repeat,col] = results[0][phy][8][repeat].x[Paramters[params]]
            task9[repeat,col] = results[0][phy][9][repeat].x[Paramters[params]]
            task10[repeat,col] = results[0][phy][10][repeat].x[Paramters[params]]
            task11[repeat,col] = results[0][phy][11][repeat].x[Paramters[params]]
        except:
            pass

# Creates 11 subplots
f1, (ax1, ax2, ax3, ax4, ax5, ax6, ax7, ax8, ax9, ax10, ax11) = plt.subplots(11, 1)
ax1.set_title('Task 1')
ax1.boxplot(task1)
ax2.set_title('Task 2')
ax2.boxplot(task2)
ax3.set_title('Task 3')
ax3.boxplot(task3)
ax4.set_title('Task 4')
ax4.boxplot(task4)
ax5.set_title('Task 5')
ax5.boxplot(task5)
ax6.set_title('Task 6')
ax6.boxplot(task6)
ax7.set_title('Task 7')
ax7.boxplot(task7)
ax8.set_title('Task 8')
ax8.boxplot(task8)
ax9.set_title('Task 9')
ax9.boxplot(task9)
ax10.set_title('Task 10')
ax10.boxplot(task10)
ax11.set_title('Task 11')
ax11.boxplot(task11)

plt.show()

#Fitness Convergence

#Each task will have a subplot
#Each subplot will have 5 lines(for each physics engine) with std
iterations = results[0][0][0][0].x_iters
task1 = np.zeros([20,iterations,5])
task2 = np.zeros([20,iterations,5])
task3 = np.zeros([20,iterations,5])
task4 = np.zeros([20,iterations,5])
task5 = np.zeros([20,iterations,5])
task6 = np.zeros([20,iterations,5])
task7 = np.zeros([20,iterations,5])
task8 = np.zeros([20,iterations,5])
task9 = np.zeros([20,iterations,5])
task10 = np.zeros([20,iterations,5])
task11 = np.zeros([20,iterations,5])


for engine in range(5):
    for repeat in range(20):
        # Task1
        true_minimum = np.min(results[0][engine][1][repeat].func_vals)
        n_calls = len(results[0][engine][1][repeat].x_iters)
        regrets = [np.sum(results[0][engine][1][repeat].func_vals[:i] - true_minimum)
                       for i in range(1, n_calls + 1)]
        task1[repeat,:,engine] = regrets
        # Task2
        true_minimum = np.min(results[0][engine][2][repeat].func_vals)
        n_calls = len(results[0][engine][2][repeat].x_iters)
        regrets = [np.sum(results[0][engine][1][repeat].func_vals[:i] - true_minimum)
                       for i in range(1, n_calls + 1)]
        task2[repeat,:,engine] = regrets
        #Task3
        true_minimum = np.min(results[0][engine][3][repeat].func_vals)
        n_calls = len(results[0][engine][3][repeat].x_iters)
        regrets = [np.sum(results[0][engine][3][repeat].func_vals[:i] - true_minimum)
                        for i in range(1, n_calls + 1)]
        task3[repeat,:,engine] = regrets
        #Task4
        true_minimum = np.min(results[0][engine][4][repeat].func_vals)
        n_calls = len(results[0][engine][4][repeat].x_iters)
        regrets = [np.sum(results[0][engine][4][repeat].func_vals[:i] - true_minimum)
                        for i in range(1, n_calls + 1)]
        task4[repeat,:,engine] = regrets 
        #Task5
        true_minimum = np.min(results[0][engine][5][repeat].func_vals)
        n_calls = len(results[0][engine][5][repeat].x_iters)
        regrets = [np.sum(results[0][engine][5][repeat].func_vals[:i] - true_minimum)
                        for i in range(1, n_calls + 1)]
        task5[repeat,:,engine] = regrets 
        #Task6
        true_minimum = np.min(results[0][engine][6][repeat].func_vals)
        n_calls = len(results[0][engine][6][repeat].x_iters)
        regrets = [np.sum(results[0][engine][6][repeat].func_vals[:i] - true_minimum)
                        for i in range(1, n_calls + 1)]
        task6[repeat,:,engine] = regrets 
        #Task7
        true_minimum = np.min(results[0][engine][7][repeat].func_vals)
        n_calls = len(results[0][engine][7][repeat].x_iters)
        regrets = [np.sum(results[0][engine][7][repeat].func_vals[:i] - true_minimum)
                        for i in range(1, n_calls + 1)]
        task7[repeat,:,engine] = regrets 
        #Task8
        true_minimum = np.min(results[0][engine][8][repeat].func_vals)
        n_calls = len(results[0][engine][8][repeat].x_iters)
        regrets = [np.sum(results[0][engine][8][repeat].func_vals[:i] - true_minimum)
                        for i in range(1, n_calls + 1)]
        task8[repeat,:,engine] = regrets 
        #Task9
        true_minimum = np.min(results[0][engine][9][repeat].func_vals)
        n_calls = len(results[0][engine][9][repeat].x_iters)
        regrets = [np.sum(results[0][engine][9][repeat].func_vals[:i] - true_minimum)
                        for i in range(1, n_calls + 1)]
        task9[repeat,:,engine] = regrets 
        #Task10
        true_minimum = np.min(results[0][engine][10][repeat].func_vals)
        n_calls = len(results[0][engine][10][repeat].x_iters)
        regrets = [np.sum(results[0][engine][10][repeat].func_vals[:i] - true_minimum)
                        for i in range(1, n_calls + 1)]
        task10[repeat,:,engine] = regrets 
        #Task11
        true_minimum = np.min(results[0][engine][11][repeat].func_vals)
        n_calls = len(results[0][engine][11][repeat].x_iters)
        regrets = [np.sum(results[0][engine][11][repeat].func_vals[:i] - true_minimum)
                        for i in range(1, n_calls + 1)]
        task11[repeat,:,engine] = regrets 

task1_mean = np.mean(task1, axis=0)
task1_std = np.std(task1, axis=0)
task2_mean = np.mean(task2, axis=0)
task2_std = np.std(task2, axis=0)
task3_mean = np.mean(task3, axis=0)
task3_std = np.std(task3, axis=0)
task4_mean = np.mean(task4, axis=0)
task4_std = np.std(task4, axis=0)
task5_mean = np.mean(task5, axis=0)
task5_std = np.std(task5, axis=0)
task6_mean = np.mean(task6, axis=0)
task6_std = np.std(task6, axis=0)
task7_mean = np.mean(task7, axis=0)
task7_std = np.std(task7, axis=0)
task8_mean = np.mean(task8, axis=0)
task8_std = np.std(task8, axis=0)
task9_mean = np.mean(task9, axis=0)
task9_std = np.std(task9, axis=0)
task10_mean = np.mean(task10, axis=0)
task10_std = np.std(task10, axis=0)
task11_mean = np.mean(task11, axis=0)
task11_std = np.std(task11, axis=0)

f2, (ax1, ax2, ax3, ax4, ax5, ax6, ax7, ax8, ax9, ax10, ax11) = plt.subplots(11, 1)
ax1.set_title('Task 1')
ax1.errorbar(range(1,n_calls+1),task1_mean,yerr=task1_std, fmt='-o')
ax2.set_title('Task 2')
ax2.errorbar(range(1,n_calls+1),task2_mean,yerr=task2_std, fmt='-o')
ax3.set_title('Task 3')
ax3.errorbar(range(1,n_calls+1),task3_mean,yerr=task3_std, fmt='-o')
ax4.set_title('Task 4')
ax4.errorbar(range(1,n_calls+1),task4_mean,yerr=task4_std, fmt='-o')
ax5.set_title('Task 5')
ax5.errorbar(range(1,n_calls+1),task5_mean,yerr=task5_std, fmt='-o')
ax6.set_title('Task 6')
ax6.errorbar(range(1,n_calls+1),task6_mean,yerr=task6_std, fmt='-o')
ax7.set_title('Task 7')
ax7.errorbar(range(1,n_calls+1),task7_mean,yerr=task7_std, fmt='-o')
ax8.set_title('Task 8')
ax8.errorbar(range(1,n_calls+1),task8_mean,yerr=task8_std, fmt='-o')
ax9.set_title('Task 9')
ax9.errorbar(range(1,n_calls+1),task9_mean,yerr=task9_std, fmt='-o')
ax10.set_title('Task 10')
ax10.errorbar(range(1,n_calls+1),task10_mean,yerr=task10_std, fmt='-o')
ax11.set_title('Task 11')
ax11.errorbar(range(1,n_calls+1),task11_mean,yerr=task11_std, fmt='-o')

#Box and Whisker Plots of Final Euclidean Error
plt.show()
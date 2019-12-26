###IMPORT LIBRARIES###
from scipy.optimize import fsolve
import math
import numpy as np
import os
import sys
import csv
from scipy.optimize import fsolve
###IMPORT CUSTOM FILES###
import Solvers

def run_sim(is4bar, theta, rover, dc = 0.1, db = 0.05, lr = 0.1, ll = 0.1):
    if is4bar is False:
        solution = fsolve(Solvers.setup_slope_matrix, (71.5,64.6,87.6,71.53,130.35,41.17,45.9,37.1,60.1,11.0,4.55,6.18,41.2,73.03,0.56), (rover, theta))
    elif is4bar is True:
        solution = fsolve(Solvers.setup_4bar_slope_matrix, (71.5,64.6,87.6,71.53,130.35,41.17,45.9,37.1,60.1,11.0,4.55,6.18,41.2,73.03,0.56), (rover, theta, db, dc, lr, ll))
    return solution

def simulate_slope_range(is4bar, rover, lower_range, upper_range):
    if os.path.exists('sloperesults.csv'):
       os.remove('sloperesults.csv')
    for theta in np.arange(lower_range, upper_range, 0.1):
         solution = run_sim(is4bar, theta, rover)
         with open('sloperesults.csv', 'a') as csvfile:
             writer = csv.writer(csvfile)
             writer.writerow([theta, solution[14]])
         if( solution[6] < 0.1):
             print("wheelie at: ", theta)
             break

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

def run_slope_sim(is4bar, theta, rover, dc = 0.1, db = 0.05, lr = 0.1, ll = 0.1):
    if is4bar is False:
        solution = fsolve(Solvers.setup_slope_matrix, (71.5,64.6,87.6,71.53,130.35,41.17,45.9,37.1,60.1,11.0,4.55,6.18,41.2,73.03,0.56), (rover, theta))
    elif is4bar is True:
        solution = fsolve(Solvers.setup_4bar_slope_matrix, (71.5,64.6,87.6,71.53,130.35,41.17,45.9,37.1,60.1,11.0,4.55,6.18,41.2,73.03,0.56), (rover, theta, db, dc, lr, ll))
    return solution

def run_obs_sim(is4bar, h, rover, dc = 0.1, db = 0.05, lr = 0.1, ll = 0.1):
    if is4bar is False:
        solution = fsolve(Solvers.setup_high_obs_matrix, (71.5,64.6,87.6,71.53,130.35,41.17,45.9,37.1,60.1,11.0,4.55,6.18,41.2,73.03,0.56), (rover, h))
    elif is4bar is True:
        solution = fsolve(Solvers.setup_4bar_high_matrix, (71.5,64.6,87.6,71.53,130.35,41.17,45.9,37.1,60.1,11.0,4.55,6.18,41.2,73.03,0.56), (rover, h, db, dc, lr, ll))
    return solution

def determine_wheelie_slope(is4bar, rover, dc = 0.1, db = 0.05, lr = 0.1, ll = 0.1):
    if os.path.exists('sloperesults.csv'):
       os.remove('sloperesults.csv')
    for theta in np.arange(0.1, 90, 0.1):
         solution = run_slope_sim(is4bar, theta, rover, dc, db, lr, ll)
         with open('sloperesults.csv', 'a') as csvfile:
             writer = csv.writer(csvfile)
             writer.writerow([theta, solution[14]])
         if( solution[6] < 0.1):
             print("wheelie at: ", theta)
             return theta
    return None

def run_full_sim(rover):
    increment = 0.01 #MODIFY THIS TO CHANGE ITERATIONS
    bogie_limit = 45 #CHANGE TO ALLOW FOR DIFFERENT LIMITS
    dc_max = 0.45 #CHANGE FOR MAX CHASSIS ANCHOR DISTANCE
    if os.path.exists('results.csv'): #FILE MANAGEMENT
        os.remove('results.csv')
    with open('results.csv', 'a') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['dc', 'db', 'lr', 'll', 'wheelie angle', 'T1', 'T2', 'T3', 'Ttotal', 'IC'])
        reg_slope = determine_wheelie_slope(False, rover)
        orig_sol = run_obs_sim(False, 0.15, rover)
        T1 = orig_sol[9]
        T2 = orig_sol[10]
        T3 = orig_sol[11]
        Tt = T1 + T2 + T3
        writer.writerow(['NA', 'NA', 'NA', 'NA', reg_slope, T1, T2, T3, Tt, 'NA'])
        for dc in np.arange(0.0, dc_max, increment):
            for db in np.arange(0.0, dc, increment):
                ls = fsolve(Solvers.bogie_limiter, (dc, dc), (bogie_limit, dc, db))
                lr = ls[0]
                ll = ls[1]
                th = determine_wheelie_slope(True, rover, dc, db, lr, ll)
                osol = run_obs_sim(True, 0.15, rover, dc, db, lr, ll)
                T11 = T1 - osol[9]
                T22 = T2 - osol[10]
                T33 = T3 - osol[11]
                Ttt = Tt - T11 - T22 - T33
                igamma = math.asin(0.5*(dc - db)/ll)
                idelta = math.asin(0.5*(dc - db)/lr)
                IC = dc/(math.tan(igamma) + math.tan(idelta))
                writer.writerow([dc, db, lr, ll, th, T11, T22, T33, Ttt, IC])

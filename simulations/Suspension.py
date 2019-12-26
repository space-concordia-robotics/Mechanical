#ARCHITECTURE OVERVIEW:
#1. DONE: Import two data files: Rover info and soil info
#   -100% complete:
#       *DONE:implement soil ini file
#2. Compute traction information from solid case
#   -99% complete:
#       *DONE: compute maximum thrust function
#       *DONE: compute compaction resistance function
#       *DONE? : compute bulldozing resistance function
#       *DONE: compute rolling resistance (use coeff of 0 if negligable) function
#       *DONE: compute gravitational resistance function
#       *DONE: create aggregate drawbar pull function
#3. Create angle geometry calculator for pivot and 4 bar cases
#   -100% complete:
#       *DONE: differentiate from obstacle mode and slope mode
#       *DONE: from rover geometry and height, get alpha and beta
#4. Create matrix calculator
#   -87% comlete:
#       *function to generate computation matrix for each scenario:
#           1. DONE: rover climbing obstacle of height h lower than radius
#           2. DONE: rover climbing obstacle of height h higher than radius
#           3. DONE: rover going across a slope
#           4. Rewrite solver for four bar cases: Halfway done
#5. create simulation to optimize rover parameters
#   -0% complete:
#       *given rover parameter and acceptable ranges, run through all instances and find ideal case for each performance parameters
#6. optimize four bar rover
#   -0% complete:
#   -Based on previous ideal parameters, using all variable 4 bar variables, find ideal case for each performance parameters
###IMPORT LIBRARIES###
from scipy.optimize import fsolve
import math
import numpy as np
import configparser
import csv
import os
import sys
#file_dir = os.path.dirname(__file__)
#sys.path.append(file_dir)
import Solvers
import Terramechanics
import Simulation
###ROVER CLASS SETUP###
class rover:
    def __init__(self, roverconfiguration, solid_wheel = True):
        self.wheel_mass = float(roverconfiguration['Weight Values (in kg)']['wheel_mass'])
        self.bogie_mass = float(roverconfiguration['Weight Values (in kg)']['bogie_mass'])
        self.wheel_link_mass = float(roverconfiguration['Weight Values (in kg)']['wheel_link_mass'])
        self.bogie_total_mass = self.bogie_mass + 2*self.wheel_link_mass
        self.body_mass = float(roverconfiguration['Weight Values (in kg)']['body_mass'])
        self.total_mass = self.wheel_mass + self.bogie_total_mass + self.body_mass
        self.coefficient_of_friction = float(roverconfiguration['Weight Values (in kg)']['coefficient_of_friction'])
        self.bogie_length = float(roverconfiguration['Lengths (in m)']['bogie_length'])
        self.wheel_link_length = float(roverconfiguration['Lengths (in m)']['wheel_link_length'])
        self.wheel_chassis_height = self.wheel_link_length + float(roverconfiguration['CG']['bogie_tube_height'])
        self.wheel_radius = float(roverconfiguration['Lengths (in m)']['wheel_radius'])
        self.wheel_width = float(roverconfiguration['Lengths (in m)']['wheel_width'])
        self.body_cg_length = float(roverconfiguration['CG']['body_cg_length'])
        self.cg_length = float(roverconfiguration['Lengths (in m)']['cg_length'])
        self.cg_height = float(roverconfiguration['Lengths (in m)']['cg_height'])
        self.poisson = float(roverconfiguration['Material']['poisson'])
        self.youngs_modulus = float(roverconfiguration['Material']['youngs_modulus'])
        self.solid_wheel = solid_wheel
        self.slope_matrix = np.zeros((15,15))
        self.low_obs_matrix = np.zeros((15,15))
        self.high_obs_matrix = np.zeros((15,15))
        self.b_slope = np.zeros(15)
        self.b_low_obs = np.zeros(15)
        self.b_high_obs = np.zeros(15)
###SOIL CLASS SETUP###
class soil:
    def __init__(self, soilconfig):
        self.n = float(soilconfig['Bekker parameters']['n'])
        self.kc = float(soilconfig['Bekker parameters']['kc'])
        self.kphi = float(soilconfig['Bekker parameters']['kphi'])
        self.c = float(soilconfig['Bekker parameters']['c'])
        self.phi = float(soilconfig['Bekker parameters']['phi'])
        self.density = float(soilconfig['Misc']['density'])
        self.poisson = float(soilconfig['Misc']['poisson'])
        self.youngs_modulus = float(soilconfig['Misc']['youngs_modulus'])

###MAIN BODY###
###INI PARSING###
rovconfig = configparser.ConfigParser()
rovconfig.sections()
rovconfig.read('Roverdata.ini')
soilconfig = configparser.ConfigParser()
soilconfig.sections()
soilconfig.read('Soil.ini')
###CONSTRUCT OBJECTS###
rover1 = rover(rovconfig, True)
mars_soil = soil(soilconfig)
##RUN SLOPE CALCS###
#Simulation.simulate_slope_range(False, rover1, 0.1, 60)
#Simulation.simulate_slope_range(True, rover1, 0.1, 60)
solution1 = Simulation.run_sim(False, 20, rover1)
solution2 = Simulation.run_sim(True, 20, rover1)
print(solution1[9], solution1[10], solution1[11])
print(solution2[9], solution2[10], solution2[11])

#for h in np.arange(0.01, rover1.wheel_radius, 0.001):
#     solution = fsolve(setup_low_obs_matrix, (71.5,64.6,87.6,71.53,130.35,41.17,45.9,37.1,60.1,11.0,4.55,6.18,41.2,73.03,0.56), (rover1, h))
#     with open('obsresults.csv', 'a') as csvfile:
#         writer = csv.writer(csvfile)
#         writer.writerow([h, solution[14]])
#for h in np.arange(0.151, rover1.wheel_radius + rover1.bogie_length, 0.001):
#     solution = fsolve(setup_high_obs_matrix, (71.5,64.6,87.6,71.53,130.35,41.17,45.9,37.1,60.1,11.0,4.55,6.18,41.2,73.03,0.56), (rover1, h))
#     with open('obsresults.csv', 'a') as csvfile:
#         writer = csv.writer(csvfile)
#         writer.writerow([h, solution[14]])
#solution = fsolve(setup_slope_matrix, (71.5,64.6,87.6,71.53,130.35,41.17,45.9,37.1,60.1,11.0,4.55,6.18,41.2,73.03,0.921), (rover1, 35))
#print("RESULTS:")
#print("NORMAL AT A:", solution[0])
#print("NORMAL AT B:", solution[1])
#print("NORMAL AT C:", solution[2])
#print("X REACTION AT A:", solution[3])
#print("X REACTION AT B:", solution[4])
#print("X REACTION AT C:", solution[5])
#print("Y REACTION AT A:", solution[6])
#print("Y REACTION AT B:", solution[7])
#print("Y REACTION AT C:", solution[8])
#print("WHEEL TORQUE AT A:", solution[9])
#print("WHEEL TORQUE AT B:", solution[10])
#print("WHEEL TORQUE AT C:", solution[11])
#print("BOGIE X REACTION:", solution[12])
#print("BOGIE Y REACTION:", solution[13])
#print("MEW:", solution[14])

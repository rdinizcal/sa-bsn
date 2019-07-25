#for computing formula
from math import *

#for reading logs
import csv 

#for plotting
import matplotlib.pyplot as plt
import numpy as np

CTX_G3_T1_3 = 1
CTX_G3_T1_4 = 1
CTX_G3_T1_1 = 1
CTX_G3_T1_2 = 1
CTX_G4_T1_1 = 1
CTX_G4_T1_3 = 1
CTX_G4_T1_2 = 1
CTX_G3_T1_X = 0
R_G3_T1_11 = 1
F_G3_T1_11 = 1
R_G3_T1_12 = 1
F_G3_T1_12 = 1
R_G3_T1_13 = 1
F_G3_T1_13 = 1
R_G3_T1_21 = 1
F_G3_T1_21 = 1
R_G3_T1_22 = 1
F_G3_T1_22 = 1
R_G3_T1_23 = 1
F_G3_T1_23 = 1
R_G3_T1_31 = 1
F_G3_T1_31 = 1
R_G3_T1_32 = 1
F_G3_T1_32 = 1
R_G3_T1_33 = 1
F_G3_T1_33 = 1
R_G3_T1_411 = 1
F_G3_T1_411 = 1
R_G3_T1_412 = 1
F_G3_T1_412 = 1
R_G3_T1_42 = 1
F_G3_T1_42 = 1
R_G3_T1_43 = 1
F_G3_T1_43 = 1
R_G3_T1_X = 1
F_G3_T1_X = 1
OPT_G3_T1_X = 1
R_G4_T1_1 = 1
F_G4_T1_1 = 1
R_G4_T1_2 = 1
F_G4_T1_2 = 1
R_G4_T1_3 = 1
F_G4_T1_3 = 1

def discretize(x,y,resolution):
    
    carry_on = 0
    s = 0
    num = 1
    pos = 0
    a = []
    b = []

    for value in x:
        value /= resolution
        if floor(value) == carry_on:
            s += y[pos]
            num += 1
        else:
            a.append(floor(value))
            b.append(s/num)
            carry_on = floor(value)
            s = y[pos]
            num = 1
        pos += 1

    return [a,b]

def function_creator(): 
  
    # expression to be evaluated 
    reliability_formula_file = open("resource/models/reliability.formula", "r")
    if reliability_formula_file.mode == 'r': reli_expr = reliability_formula_file.read()
    else : exit()

    # variable used in expression
    with open("resource/logs/1563994960139564028.log", newline='') as log_file:
        log_csv = csv.reader(log_file, delimiter=',')
        log = list(log_csv)
        del log[0]

    timeseries = [] 
    t0 = int(log[1][1])
    for registry in log:
        var = registry[2].replace(".","_")
        exec("C_"+ var + " = " + registry[3])
        exec("R_"+ var + " = " + registry[4])
        exec("F_"+ var + " = " + registry[5])
        instant = int(registry[1]) - t0
        timeseries.append([instant, eval(reli_expr)])
    
    #plot timeseries
    x = [x[0] for x in timeseries]
    y = [y[1] for y in timeseries]
    #print(x)

    # discretizing the curve resolution = 1sec
    [x,y] = discretize(x,y,10e9)

    plt.plot(x, y)
    plt.ylim(0.8,1)
    plt.xlim(0,max(x))
    plt.show()
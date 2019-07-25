#for computing formula
from math import *

#for reading logs
import csv 

#for plotting
import matplotlib.pyplot as plt
import numpy as np

#for splitting
import re

#for curve analysis
from statistics import mean

class Analyzer:

    def __init__(self): pass
        
    # computes the average for evey truncated (which dependes on the resolution) x value 
    # [38.1, 38.5, 38.7]:[5, 10, 15] returns [38,15]
    def discretize(self, x, y, resolution):
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

    def analyze(self, x, y, setpoint, upper_bound, lower_bound):

        # calculate stability point
        pos = 0
        flag = False
        for value in y:
            if lower_bound <= value and value <= upper_bound:
                if flag is False:
                    stability_point = pos
                    flag = True
            else:
                stability_point = 0
                flag = False

            pos+=1

        print('Stability: %r' % bool(stability_point is not 0))

        #calculate settling time
        settling_time = x[stability_point] - x[0]
        print('Settling Time: %s' % settling_time)

        #calculate overshoot
        overshoot = 100*(max(y) - setpoint)/setpoint
        print('Overshoot: %.2f%%' % overshoot)

        #calculate steady-state error
        sse = 100*(abs(setpoint - mean(val for val in y[stability_point:]))/setpoint)
        print('Steady-State Error: %.2f%%' % sse)

        #controller effort, not engough information yet

        #robustness
        robustness = 100*(1 - sum([abs(setpoint - val) for val in y])/len(x))
        print('Robustness: %.2f%%' % robustness)

    def run(self): 
        # load formula
        formula = Formula("resource/models/reliability.formula")

        # load log
        with open("resource/logs/1564085006280868817.log", newline='') as log_file:
            log_csv = csv.reader(log_file, delimiter=',')
            log = list(log_csv)
            del log[0] # delete first line

        # compute time serie
        timeseries = [] 
        t0 = int(log[0][1])
        for registry in log:
            var = registry[2].replace(".","_")
            formula.compute(var, registry[3])
            instant = int(registry[1]) - t0
            timeseries.append([instant, formula.eval()])
        
        x = [x[0] for x in timeseries]
        y = [y[1] for y in timeseries]

        # discretizing the curve
        [x,y] = self.discretize(x,y,10e7)

        # perform the analysis 
        setpoint = 0.80
        self.analyze(x,y, setpoint, setpoint*1.05, setpoint*0.95)

        # plot timeseries
        plt.plot(x, y)
        plt.ylim(0.8,1)
        plt.xlim(0,max(x))
        plt.show()

class Formula:

    def __init__(self, path): 
        formula_file = open(path, 'r')
        if formula_file.mode == 'r': 
            self.expression = formula_file.read()
            self.mapping = self.initialize_expr()
        else : 
            raise Exception('Formula file not found')
    
    def initialize_expr(self):
        expr = self.expression.replace("*"," ")
        expr = expr.replace("+"," ")
        expr = expr.replace("-"," ")
        expr = expr.replace("/"," ")
        expr = expr.replace("("," ")
        expr = expr.replace(")"," ")
        expr = re.split(' ',expr)
        arguments = list(filter(None, expr))

        arg_val = {}
        for argument in arguments :
            arg_val[argument] = 1

        return arg_val

    def compute(self, arg, value):
        self.mapping[arg] = float(value)

    def eval(self):
        return eval(self.expression, self.mapping)

        

    


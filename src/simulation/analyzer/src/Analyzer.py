#for computing formula
from math import *

#for reading logs
import csv 

#for plotting
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.ticker as ticker

#for splitting
import re

#for curve analysis
from statistics import mean

#for ordered dict
from collections import OrderedDict

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
        formula = Formula("../../knowledge_repository/resource/models/reliability.formula")

        # build list of participating tasks
        tasks = dict()
        ctxs = dict()

        global_reli_timeseries = dict() 

        ################ load status log ################
        with open("../../knowledge_repository/resource/logs/status_1569174520555742018.log", newline='') as log_file:
            log_csv = csv.reader(log_file, delimiter=',')
            log_status = list(log_csv)
            del log_status[0] # delete first line

        ################ load event log ################
        with open("../../knowledge_repository/resource/logs/event_1569174520555742018.log", newline='') as log_file:
            log_csv = csv.reader(log_file, delimiter=',')
            log_event = list(log_csv)
            del log_event[0] # delete first line

        ################ load uncertainty log ################
        with open("../../knowledge_repository/resource/logs/uncertainty_1569174520555742018.log", newline='') as log_file:
            log_csv = csv.reader(log_file, delimiter=',')
            log_uncert = list(log_csv)
            del log_uncert[0] # delete first line

        #concatenate lists into one log list
        log = list()
        log.extend(log_status)
        log.extend(log_event)

        log = sorted(log, key = lambda x: (int(x[1])))

        t0 = int(log[0][2])

        # read log 
        for reg in log:
            if(reg[0]=="Status"):
                status = Status(str(reg[1]),str(reg[2]),str(reg[3]),str(reg[4]),str(reg[5]))

                tag = status.source.upper().replace(".","_").replace("/","").replace("T","_T")

                if not (tag in tasks): 
                    tsk = Task(tag)
                    tasks[tag] = tsk

                if (status.content == 'success'):
                    tasks[tag].execute() 
                    tasks[tag].success()
                elif (status.content == 'fail'): 
                    tasks[tag].execute() 
                    tasks[tag].fail()

            elif(reg[0]=="Event"):
                event = Event(str(reg[1]),str(reg[2]),str(reg[3]),str(reg[4]),str(reg[5]))

                tag = event.source.upper().replace(".","_").replace("/","").replace("T","_T")

                if not (tag in ctxs): 
                    ctx = Context(tag)
                    ctxs[tag] = ctx
                
                if (event.content == 'deactivate'): ctxs[tag].deactivate()
                elif (event.content == 'activate'): ctxs[tag].activate()

            # compute time series
            instant = int(reg[2]) - t0
            ## global reliability timeseries
            for task in tasks.values():
                formula.compute('R_'+task.getName(), task.reliability())
                formula.compute('C_'+task.getName(), task.cost())
                formula.compute('F_'+task.getName(), task.frequency())

            for ctx in ctxs.values():
                formula.compute('CTX_'+ctx.getName(), ctx.isActive())

            global_reli_timeseries[instant] = formula.eval()

        input_timeseries = dict()
        noise_factor = dict()

        for reg in log_uncert:
            instant = (int(reg[2]) - t0)
            tag = reg[4].upper().replace(".","_").replace("/","").replace("T","_T")
            noise = float(reg[5].split("=")[1])

            if not (tag in input_timeseries): 
                input_timeseries[tag] = [[0,0]]
                input_timeseries[tag] = [[instant,noise]]

            if not (tag in noise_factor): 
                noise_factor[tag] = 0

            input_timeseries[tag].append([instant-1,noise_factor[tag]])
            input_timeseries[tag].append([instant,noise])
            noise_factor[tag] = noise

        # plot global reli timeseries
        x = list(global_reli_timeseries.keys())
        y = list(global_reli_timeseries.values())

        ## discretizing the curve
        [x,y] = self.discretize(x,y,1) #precision in ms

        ## perform the analysis 
        setpoint = 0.80
        self.analyze(x, y, setpoint, setpoint*1.05, setpoint*0.95)

        scale = 10e-10
        ticks = ticker.FuncFormatter(lambda x, pos: '{0:g}'.format(x*scale))

        ## plot timeseries
        fig, ax = plt.subplots()
        ax.plot(x, y)
        ax.set_ylim(0,1.05)
        ax.xaxis.set_major_formatter(ticks)
        #ax.axvline(x=0.7*10e10, linestyle='--', color='red')
        #ax.axvline(x=0.9*10e10, linestyle='--', color='red')

        fig.suptitle('System reliability in time')
        fig.text(0.04, 0.5, 'Reliability (%)', va='center', rotation='vertical')
        fig.text(0.5, 0.04, 'Time (s)', ha='center')
        plt.grid()
        
        fig, axs = plt.subplots(len(input_timeseries),1, sharex='all')
        fig.suptitle('Noise in sensing')
        if(len(input_timeseries) > 1):
            i = 0
            for tag in input_timeseries:
                x = [el[0] for el in input_timeseries[tag]]
                y = [el[1] for el in input_timeseries[tag]]

                axs[i].plot(x,y)
                axs[i].set_ylabel(tag, fontsize=8)
                axs[i].xaxis.set_visible(False)
                #axs[i].axvline(x=0.7*10e10, linestyle='--', color='red')
                #axs[i].axvline(x=0.9*10e10, linestyle='--', color='red')

                i += 1
            axs[len(input_timeseries)-1].xaxis.set_visible(True)
            axs[len(input_timeseries)-1].xaxis.set_major_formatter(ticks)
        else:
            x = [el[0] for el in input_timeseries[tag]]
            y = [el[1] for el in input_timeseries[tag]]

            axs.plot(x,y)
            axs.set_ylabel(tag, fontsize=8)
            axs.xaxis.set_major_formatter(ticks)
        
        fig.text(0.04, 0.5, 'Noise factor injected', va='center', rotation='vertical')
        fig.text(0.5, 0.04, 'Time (s)', ha='center')

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
            arg_val[argument] = 0

        return arg_val

    def compute(self, arg, value):
        self.mapping[arg] = float(value)

    def eval(self):
        return eval(self.expression, self.mapping)

class ReconfigurationCommand:

    def __init__(self, _logical_instant, _instant, _source, _target, _action): 
        self.source = _source
        self.target = _target
        self.logical_instant = _logical_instant
        self.instant = _instant
        self.action = _action

class Status:

    def __init__(self, _logical_instant, _instant, _source, _target, _content): 
        self.source = _source
        self.target = _target
        self.logical_instant = _logical_instant
        self.instant = _instant
        self.content = _content

class Event:

    def __init__(self, _logical_instant, _instant, _source, _target, _content): 
        self.source = _source
        self.target = _target
        self.logical_instant = _logical_instant
        self.instant = _instant
        self.content = _content
    
class Task:

    def __init__(self, _name):
        self.name = _name 
        self.nexecs = 0
        self.lstExec = []
        self.window_size = 100
    
    def __eq__(self, other):
        if isinstance(other, Task):
            return self.name == other.name
        return NotImplemented

    def __hash__(self):
        return hash(tuple(sorted(self.__dict__.items())))

    def reliability (self):
        if self.nexecs != 0: return sum(self.lstExec) / self.nexecs # Success/Fail+Success
        else: return 0
    
    def cost (self) : return 1
    
    def frequency(self): return 1

    def execute(self) :
        if(self.nexecs < self.window_size) : self.nexecs += 1
    
    def success(self) :
        if(self.nexecs < self.window_size) : self.lstExec.append(1)
        else: 
            self.lstExec.append(1)
            del self.lstExec[0]

    def fail(self) : 
        if(self.nexecs < self.window_size) : self.lstExec.append(0)
        else: 
            self.lstExec.append(0)
            del self.lstExec[0]
    
    def getName(self) :
        return self.name

class Context:

    def __init__(self, _name):
        self.name = _name 
        self.active = 1
    
    def __eq__(self, other):
        if isinstance(other, Task):
            return self.name == other.name
        return NotImplemented

    def __hash__(self):
        return hash(tuple(sorted(self.__dict__.items())))

    def getName(self) :
        return self.name

    def isActive (self):
        return self.active
    
    def activate (self) :
        self.active = 1
    
    def deactivate(self):
        self.active = 0

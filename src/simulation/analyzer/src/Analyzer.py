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
        with open("resource/logs/1564600009534118019.log", newline='') as log_file:
            log_csv = csv.reader(log_file, delimiter=',')
            log = list(log_csv)
            del log[0] # delete first line

        # build list of participating tasks
        tasks = dict()
        ctxs = dict()
        ctxs_timeseries = dict()
        reli_timeseries = dict() 
        global_reli_timeseries = [] 
        t0 = int(log[0][2])

        # read log 
        for reg in log:
            if (reg[0] == 'ReconfigurationCommand' ) :  
                command = ReconfigurationCommand(str(reg[1]),str(reg[2]),str(reg[3]),str(reg[4]),str(reg[5]))

                tag = command.target.upper().replace(".","_").replace("/","").replace("T","_T")
                
                if not (tag in tasks): 
                    tsk = Task(tag)
                    tasks[tag] = tsk

                if (command.action == 'execute'): tasks[tag].execute()
                
            elif (reg[0] == 'Status' ) :  
                status = Status(str(reg[1]),str(reg[2]),str(reg[3]),str(reg[4]),str(reg[5]),str(reg[6]))

                tag = status.key

                if not (tag in ctxs): 
                    ctx = Context(tag)
                    ctxs[tag] = ctx
                
                if (int(status.value) == 1): ctxs[tag].activate()
                else: ctxs[tag].deactivate()

            elif (reg[0] == 'Event' ) :  
                #lstEvent.append(Event(str(reg[1]),str(reg[2]),str(reg[3]),str(reg[4]),str(reg[5]),str(reg[6])))
                event = Event(str(reg[1]),str(reg[2]),str(reg[3]),str(reg[4]),str(reg[5]),str(reg[6]))

                tag = event.source.upper().replace(".","_").replace("/","").replace("T","_T")

                if not (tag in tasks): 
                    tsk = Task(tag)
                    tasks[tag] = tsk

                if (event.type == 'failure'): tasks[tag].fail()

            # compute time series
            instant = int(reg[2]) - t0
            ## global reliability timeseries
            for task in tasks.values():
                formula.compute('R_'+task.getName(), task.reliability())
                formula.compute('C_'+task.getName(), task.cost())
                formula.compute('F_'+task.getName(), task.frequency())
            
            for ctx in ctxs.values():
                formula.compute(ctx.getName(), ctx.isActive())

            global_reli_timeseries.append([instant, formula.eval()])

            ## contexts timeseries
            for tag in ctxs:
                if tag in ctxs_timeseries: ctxs_timeseries[tag].append([instant, ctxs[tag].isActive()])
                else: ctxs_timeseries[tag] = [[instant, ctxs[tag].isActive()]]

            ## individual reliability timeseries


        # plot global reli timeseries
        x = [x[0] for x in global_reli_timeseries]
        y = [y[1] for y in global_reli_timeseries]

        ## discretizing the curve
        [x,y] = self.discretize(x,y,10e7)

        ## perform the analysis 
        setpoint = 0.80
        self.analyze(x,y, setpoint, setpoint*1.05, setpoint*0.95)

        ## plot timeseries
        plt.plot(x, y)
        #plt.ylim(0.8,1.1)
        #plt.xlim(0,max(x))

        # plot contexts timeseries
        x = dict()
        y = dict()
        ctx = ""
        for context,timeserie in ctxs_timeseries.items():
            x[context] = [x[0] for x in timeserie]
            y[context] = [y[1] for y in timeserie]

            ## discretizing the curve
            [x[context],y[context]] = self.discretize(x[context], y[context],10e7)
            ctx = context

        ## plot timeseries
        #plt.title("Active contexts")
        plt.plot(x[ctx], [sum(x) for x in zip(*y.values())])
        #plt.ylim(0,len(ctxs_timeseries.keys()))
        #plt.xlim(0,max(x))
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

class ReconfigurationCommand:

    def __init__(self, _logical_instant, _instant, _source, _target, _action): 
        self.source = _source
        self.target = _target
        self.logical_instant = _logical_instant
        self.instant = _instant
        self.action = _action

class Status:

    def __init__(self, _logical_instant, _instant, _source, _target, _key, _value): 
        self.source = _source
        self.target = _target
        self.logical_instant = _logical_instant
        self.instant = _instant
        self.key = _key
        self.value = _value

class Event:

    def __init__(self, _logical_instant, _instant, _source, _target, _type, _description): 
        self.source = _source
        self.target = _target
        self.logical_instant = _logical_instant
        self.instant = _instant
        self.type = _type
        self.description = _description
    
class Task:

    def __init__(self, _name):
        self.name = _name 
        self.nexecs = 0
        self.nfails = 0
    
    def __eq__(self, other):
        if isinstance(other, Task):
            return self.name == other.name
        return NotImplemented

    def __hash__(self):
        return hash(tuple(sorted(self.__dict__.items())))

    def reliability (self):
        if self.nexecs != 0: return (self.nexecs - self.nfails) / self.nexecs 
        else: return 0
    
    def cost (self) :
        return 1
    
    def frequency(self):
        return 1

    def execute(self) :
        self.nexecs += 1
    
    def fail(self) : 
        self.nfails += 1
    
    def getName(self) :
        return self.name

    def getExecs(self) :
        return self.nexecs
    
    def getFails(self) :
        return self.nfails

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

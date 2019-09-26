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

    def __init__(self, argc, argv):
        self.file_id = argv[1]
        self.formula_id = argv[2]
        self.stability = False
        self.settling_time = 0
        self.overshoot = 0
        self.sse = 0
        self.robustness = 0
        
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

        self.stability = bool(stability_point is not 0)
        print('Stability: %r' % self.stability)

        #calculate settling time
        self.settling_time = float(x[stability_point] - x[0])/10e9
        print('Settling Time: %.2fs' % self.settling_time)

        #calculate overshoot
        self.overshoot = 100*(max(y) - setpoint)/setpoint
        print('Overshoot: %.2f%%' % self.overshoot)

        #calculate steady-state error
        self.sse = 100*(abs(setpoint - mean(val for val in y[stability_point:]))/setpoint)
        print('Steady-State Error: %.2f%%' % self.sse)

        #controller effort, not engough information yet

        #robustness
        self.robustness = 100*(1 - sum([abs(setpoint - val) for val in y])/len(x))
        print('Robustness: %.2f%%' % self.robustness)

    def run(self): 
        # load formula
        formula = Formula("../../knowledge_repository/resource/models/"+self.formula_id+".formula")

        # build list of participating tasks
        tasks = dict()
        ctxs = dict()

        global_reli_timeseries = dict() 
        local_reli_timeseries = dict()
        local_status_timeseries = dict()

        ################################################################## 
        #                                                                #
        #                   Load Registries In Memory                    #
        #                                                                #
        ################################################################## 

        ################ load status log ################
        with open("../../knowledge_repository/resource/logs/status_" + self.file_id + ".log", newline='') as log_file:
            log_csv = csv.reader(log_file, delimiter=',')
            log_status = list(log_csv)
            del log_status[0] # delete first line

        ################ load event log ################
        with open("../../knowledge_repository/resource/logs/event_" + self.file_id + ".log", newline='') as log_file:
            log_csv = csv.reader(log_file, delimiter=',')
            log_event = list(log_csv)
            del log_event[0] # delete first line

        ################ load uncertainty log ################
        with open("../../knowledge_repository/resource/logs/uncertainty_" + self.file_id + ".log", newline='') as log_file:
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
            # compute time series
            instant = int(reg[2]) - t0

            if(reg[0]=="Status"):
                status = Status(str(reg[1]),str(reg[2]),str(reg[3]),str(reg[4]),str(reg[5]))

                tag = status.source.upper().replace(".","_").replace("/","").replace("T","_T")

                if not (tag in tasks): 
                    tsk = Task(tag)
                    tasks[tag] = tsk

                if (status.content == 'success'): tasks[tag].success()
                elif (status.content == 'fail'): tasks[tag].fail()

                if (status.content == 'success' or status.content == 'fail'):
                    if not (tag in local_status_timeseries):
                        local_status_timeseries[tag] = [[0,0]]
                        local_status_timeseries[tag] = [[instant,status.content]]

                    local_status_timeseries[tag].append([instant,status.content])

                if not (tag in local_reli_timeseries): 
                    local_reli_timeseries[tag] = [[0,0]]
                    local_reli_timeseries[tag] = [[instant,tasks[tag].reliability()]]

                local_reli_timeseries[tag].append([instant,tasks[tag].reliability()])


            elif(reg[0]=="Event"):
                event = Event(str(reg[1]),str(reg[2]),str(reg[3]),str(reg[4]),str(reg[5]))

                tag = event.source.upper().replace(".","_").replace("/","").replace("T","_T")

                if not (tag in ctxs): 
                    ctx = Context(tag)
                    ctxs[tag] = ctx
                
                if (event.content == 'deactivate'): ctxs[tag].deactivate()
                elif (event.content == 'activate'): ctxs[tag].activate()

            ## global reliability timeseries
            for task in tasks.values():
                formula.compute('R_'+task.getName(), task.reliability())
                formula.compute('C_'+task.getName(), task.cost())
                formula.compute('F_'+task.getName(), task.frequency())

                #print('R_'+task.getName()+"= "+str(task.reliability()))

            for ctx in ctxs.values():
                formula.compute('CTX_'+ctx.getName(), ctx.isActive())

            global_reli_timeseries[instant] = formula.eval()
            #print(str(instant) +". "+ str(formula.eval()))

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


        ################################################################## 
        #                                                                #
        #         Perform Control Theoretical Analysis Timeseries        #
        #                                                                #
        ################################################################## 

        x = list(global_reli_timeseries.keys())
        y = list(global_reli_timeseries.values())

        ## discretizing the curve
        [x,y] = self.discretize(x,y,1) #precision in ms

        setpoint = 0.9
        upper_margin = setpoint*1.05
        lower_margin = setpoint*0.95
        self.analyze(x, y, setpoint, setpoint*1.05, setpoint*0.95)

        ################################################################## 
        #                                                                #
        #                         Plot Timeseries                        #
        #                                                                #
        ################################################################## 

        scale = 10e-10
        ticks = ticker.FuncFormatter(lambda x, pos: '{0:g}'.format(x*scale))

        default_colors = ["#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#17becf"]
        colors = dict()
        i = 0
        for tag in tasks:
            colors[tag] = default_colors[i]
            i+=1

        ## First, plot the global reliability against time
        fig, ax = plt.subplots()
        ax.plot(x, y, label='BSN', color = "#1f77b4", linewidth=2)
        ax.set_ylim(0,1.05)
        ax.xaxis.set_major_formatter(ticks)

        ## Then, plot the local reliabilities against time (same figure)
        i = 0
        for tag in local_reli_timeseries:
            x = [el[0] for el in local_reli_timeseries[tag]]
            y = [el[1] for el in local_reli_timeseries[tag]]

            ax.plot(x, y, label=tag, color=colors[tag])

        ## Plot horizontal lines for setpoint
        ax.axhline(y=setpoint, linestyle='--', linewidth=1.0, color="black")
        ax.axhline(y=upper_margin, linestyle='--', linewidth=0.5, color="black")
        ax.axhline(y=lower_margin, linestyle='--', linewidth=0.5, color="black")

        ## Insert labels, titles, texts, grid...
        fig.suptitle('Reliability behavior in time')
        is_stable = "stable" if self.stability else "not stable" 
        subtitle = '%s | ST = %.2fs | OS = %.2f%% | SSE = %.2f%%' % (is_stable,self.settling_time,self.overshoot,self.sse)        
        fig.text(0.5, 0.90, subtitle, ha='center', color = "grey") # subtitle
        fig.text(0.04, 0.5, 'Reliability (%)', va='center', rotation='vertical')
        fig.text(0.5, 0.035, 'Time (s)', ha='center')
        fig.text(0.8, 0.020, self.file_id + '.log', ha='center', color = "grey", fontsize=6) # files used
        fig.text(0.8, 0.005, self.formula_id + '.formula', ha='center', color = "grey", fontsize=6) # files used
        plt.grid()
        plt.legend()
        
        ## Second, plot the local uncertainty inputs against time (second figure)
        fig, ax = plt.subplots()
        ax.set_ylabel(tag, fontsize=8)
        ax.set_ylim(0,1.05)
        ax.xaxis.set_major_formatter(ticks)

        for tag in input_timeseries:
            x = [el[0] for el in input_timeseries[tag]]
            y = [el[1] for el in input_timeseries[tag]]

            ax.plot(x,y, label=tag, color=colors[tag])

        fig.suptitle('Uncertainty injection inputs')
        fig.text(0.04, 0.5, 'Noise factor injected', va='center', rotation='vertical')
        fig.text(0.5, 0.035, 'Time (s)', ha='center')
        plt.grid()
        plt.legend()

        ## Third, plot in horizontal bars the failures and successes of each component at a time (third figure)

        # First I have to discretize the curves
        discretized_status_timeseries = dict()
        res = 10e9 # resolution in seconds

        x_max = 0
        for tag in local_status_timeseries:
            last = len(local_status_timeseries[tag]) - 1
            x_max  = local_status_timeseries[tag][last][0] if local_status_timeseries[tag][last][0] > x_max else x_max

        for tag in local_status_timeseries:
            lst = local_status_timeseries[tag] # a list of pairs [instant,"status"] for that tag

            if not (tag in discretized_status_timeseries): # initialize dict
                discretized_status_timeseries[tag] = [[lst[0][0], 1 if lst[0][1] == 'success' else 0]]

            instant = 0 
            aux = list()
            flag = True
            for pair in lst:

                if flag : 
                    instant = pair[0]
                    flag = False

                if(pair[0] < instant + res):
                    aux.append(pair)
                else:
                    success = 0
                    fail = 0
                    for pair in aux:
                        if pair[1] == "success" : success += 1
                        elif pair[1] == "fail" : fail+=1
                    
                    val = (1 if success/(success+fail) > 0.80 else 0) if success+fail > 0 else -1
                    discretized_status_timeseries[tag].append([instant, val])
                    aux.clear()
                    flag = True
        
        #calculate whole system reliability based on the discretized status timeseries and append
        bsn_tag = "BSN"
        discretized_status_timeseries[bsn_tag] = [[0,0]]
        steps = list(range(0, int(x_max), int(res)))
        last = len(steps)-1
        steps[last] = x_max # fix last value due to transformation to integer (mandatory)

        x = 0
        for step in steps:
            val = 0
            for pair in discretized_status_timeseries["G3_T1_1"]:
                if pair[0] >= x and pair[0] < step :
                    val += pair[1]
                    break 
            
            for pair in discretized_status_timeseries["G3_T1_2"]:
                if pair[0] >= x and pair[0] < step :
                    val += pair[1]
                    break 
                    
            for pair in discretized_status_timeseries["G3_T1_3"]:
                if pair[0] >= x and pair[0] < step :
                    val += pair[1]
                    break 
            
            for pair in discretized_status_timeseries["G3_T1_4"]:
                if pair[0] >= x and pair[0] < step :
                    val += pair[1]
                    break 

            for pair in discretized_status_timeseries["G4_T1"]:
                if pair[0] >= x and pair[0] < step :
                    val *= pair[1]
                    break 
            
            discretized_status_timeseries[bsn_tag].append([step, 1 if val > 0 else 0])
            x = step

        #Then plot horizontal lines
        scalez = int(x_max) * 10e-10
        ticksz = ticker.FuncFormatter(lambda x, pos: '{0:g}'.format(x*scalez))
        fig, ax = plt.subplots()
        ax.xaxis.set_major_formatter(ticksz)

        cc = {1:"green", 0:"red", -1:"white"}
        for tag in discretized_status_timeseries:
            x = discretized_status_timeseries[tag][0][0]
            #last = len(discretized_status_timeseries[tag])-1
            #x_max = discretized_status_timeseries[tag][last][0]
            for pair in discretized_status_timeseries[tag]:
                ax.axhline(y=tag, xmin=x/x_max, xmax=pair[0]/x_max, linewidth=5.0, color=cc[pair[1]])
                x = pair[0]
        
        fig.suptitle('Success/Failure map in time')
        fig.text(0.5, 0.035, 'Time (s)', ha='center')

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
        self.lstExec = []
        self.window_size = 250
    
    def __eq__(self, other):
        if isinstance(other, Task):
            return self.name == other.name
        return NotImplemented

    def __hash__(self):
        return hash(tuple(sorted(self.__dict__.items())))

    def reliability (self):
        if len(self.lstExec) > 10: return sum(self.lstExec) / len(self.lstExec) # Success/Fail+Success
        else: return 0
    
    def cost (self) : return 1
    
    def frequency(self): return 1
    
    def success(self) :
        self.lstExec.append(1)
        if(len(self.lstExec) > self.window_size) : 
            del self.lstExec[0] 

    def fail(self) : 
        self.lstExec.append(0)
        if(len(self.lstExec) > self.window_size) : 
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

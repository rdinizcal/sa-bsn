#!/usr/bin/env python

import sys
#for computing formula
from math import *

import rospy
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

from archlib.msg import Strategy
from archlib.msg import Persist
from services.srv import Address
from services.srv import EnactorInfo

import time
import rospkg

class Analyzer:

    def __init__(self, argc, argv):
        self.repository_path = rospkg.RosPack().get_path('repository')
        self.file_id = self.receive_file_id()
        self.formula_id = "reliability"
        self.stability_margin = 0.03
        self.stability = False
        self.convergence_point = 0
        self.settling_time = 0
        self.overshoot = 0
        self.sse = 0
        self.robustness = 0
        self.logical_clock = 0
        self.received_command = False
        self.pub = rospy.Publisher('persist', Persist, queue_size=10)
        self.a = rospy.Subscriber("strategy", Strategy, self.callback)
        self.enactor_kp = 0
        
    def analyze(self, x, y, setpoint):

        print('-----------------------------------------------')
        self.convergence_point = mean(val for val in y[int(3*len(x)/4):]) # last quarter of the curve, lets hope it sufficses
        print('Converge to: %.2f' % self.convergence_point)

        lower_bound = self.convergence_point*(1-self.stability_margin)
        upper_bound = self.convergence_point*(1+self.stability_margin)

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
        self.settling_time = float(x[stability_point] - x[0])/10e8
        print('Settling Time: %.2fs' % self.settling_time)

        #calculate overshoot
        #y_max = max(yi for yi in y[:stability_point])
        self.overshoot = 100*(max(y) - self.convergence_point)/self.convergence_point
        print('Overshoot: %.2f%%' % self.overshoot)

        #calculate steady-state error
        self.sse = 100*(abs(setpoint - self.convergence_point)/setpoint)
        print('Steady-State Error: %.2f%%' % self.sse)

        #controller effort, not enough information yet

        #robustness
        self.robustness = 100*(1 - sum([abs(setpoint - val) for val in y])/len(x))
        print('Robustness: %.2f%%' % self.robustness)

        print('-----------------------------------------------')
    
    def receive_file_id(self):
        rospy.wait_for_service('address')
        receive_id = rospy.ServiceProxy('address', Address)
        resp = receive_id()
        
        return resp.id

    def receive_enactor_info(self):
        rospy.wait_for_service('enactor_info')
        receive_info = rospy.ServiceProxy('enactor_info', EnactorInfo)
        resp = receive_info()

        self.enactor_kp = resp.kp
        print("Enactor Kp: " + self.enactor_kp)

    def callback(self, data):
        with open(self.repository_path + "/../resource/logs/status_" + self.file_id + "_tmp.log", 'w') as log_file:
            log_file.truncate()
            log_file.close()

        #with open(self.repository_path + "/../resource/logs/event_" + self.file_id + "_tmp.log", 'w') as log_file:
            #log_file.truncate()
            #log_file.close()
            
        self.received_command = True

    def run(self):
        self.receive_enactor_info()
        rospy.init_node("analyzer")
        
        loop_rate = rospy.Rate(1)
        loop_rate.sleep()

        while not rospy.is_shutdown():
            if self.received_command:
                self.logical_clock += 1
            
            if self.logical_clock == 20: #150
                self.body()
                self.received_command = False
                self.logical_clock = 0
            
            loop_rate.sleep()

    def body(self):
        # load formula
        formula = Formula(self.repository_path + "/../resource/models/"+self.formula_id+".formula", "float")

        # build list of participating tasks
        tasks = dict()
        ctxs = dict()

        global_reli_timeseries = dict() 
        global_status_timeseries = dict()

        ################################################################## 
        #                                                                #
        #                   Load Registries In Memory                    #
        #                                                                #
        ################################################################## 

        ################ load status log ################
        with open(self.repository_path + "/../resource/logs/status_" + self.file_id + "_tmp.log", mode='r') as log_file:
            status_lines = log_file.readlines()

        with open(self.repository_path + "/../resource/logs/status_" + self.file_id + "_tmp.log", mode='rb') as log_file:
            log_csv = csv.reader(log_file, delimiter=',')
            log_status = list(log_csv)
            #del log_status[0] # delete first line (do we need this?)

        ################ load event log ################    
        with open(self.repository_path + "/../resource/logs/event_" + self.file_id + "_tmp.log", mode='rb') as log_file:
            event_lines = log_file.readlines()

        with open(self.repository_path + "/../resource/logs/event_" + self.file_id + "_tmp.log", mode='rb') as log_file:
            log_csv = csv.reader(log_file, delimiter=',')
            log_event = list(log_csv)
            #del log_event[0] # delete first line (do we need this?)

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

                if (status.content == 'success'): tasks[tag].success(instant)
                elif (status.content == 'fail'): tasks[tag].fail(instant)

                if (status.content == 'success' or status.content == 'fail'):
                    ## compute global formulae
                    for tag in tasks:
                        formula.compute('R_'+tag, tasks[tag].reliability())
                        formula.compute('C_'+tag, tasks[tag].cost())
                        formula.compute('F_'+tag, tasks[tag].frequency())

            elif(reg[0]=="Event"):
                event = Event(str(reg[1]),str(reg[2]),str(reg[3]),str(reg[4]),str(reg[5]))

                tag = event.source.upper().replace(".","_").replace("/","").replace("T","_T")

                if not (tag in ctxs): 
                    ctx = Context(tag)
                    ctxs[tag] = ctx
                
                if (event.content == 'deactivate'): ctxs[tag].deactivate()
                elif (event.content == 'activate'): ctxs[tag].activate()

                for ctx in ctxs.values():
                    formula.compute('CTX_'+ctx.getName(), ctx.isActive())


            if(reg[0]=="Event" or reg[0]=="Status"):
                global_reli_timeseries[instant] = formula.eval()

        ################################################################## 
        #                                                                #
        #         Perform Control Theoretical Analysis Timeseries        #
        #                                                                #
        ################################################################## 
        x = list(global_reli_timeseries.keys())
        y = list(global_reli_timeseries.values())

        setpoint = 0.95

        xa = []
        ya = []
        i = 0
        for xi in x:
            xa.append(x[i])
            ya.append(y[i])
            i += 1

        self.analyze(x, y, setpoint)
        
        msg = Persist()
        msg.source = "analyzer"
        msg.target = "data access"
        content = str(self.enactor_kp) + ";"
        content += str(self.stability) + ";" 
        content += str(self.convergence_point) + ";"
        content += str(self.settling_time) + ";"
        content += str(self.overshoot) + ";"
        content += str(self.sse)
        msg.content = content
        msg.type = "ControlTheoryMetrics"
        msg.timestamp = time.time()
        self.pub.publish(msg)

        with open(self.repository_path + "/../resource/logs/analyzed_data_" + self.file_id + ".log", mode='a+') as persist_file:
            persist_file.write("Events\n")
            persist_file.writelines(event_lines)
            persist_file.write("Status\n")
            persist_file.writelines(status_lines)
            persist_file.write("\n")

class Formula:

    def __init__(self, path, _type): 
        self.type = _type
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
        expr = expr.replace("and"," ")
        expr = expr.replace("or"," ")
        expr = re.split(' ',expr)
        arguments = list(filter(None, expr))

        arg_val = {}
        for argument in arguments :
            if self.type == "bool": arg_val[argument] = False
            else : arg_val[argument] = 0

        return arg_val

    def compute(self, arg, value):
        if self.type == "bool": self.mapping[arg] = bool(value)
        else: self.mapping[arg] = float(value)
    
    def eval(self):
        return eval(self.expression, self.mapping)

class AdaptationCommand:

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
        self.lstInvocations = list()
        self.res = 1 * 10e9 #seconds in nanoseconds
    
    def __eq__(self, other):
        if isinstance(other, Task):
            return self.name == other.name
        return NotImplemented

    def __hash__(self):
        return hash(tuple(sorted(self.__dict__.items())))

    def reliability (self):
        last = len(self.lstInvocations)-1
        if(last <= 0): return 0
        last_step = self.lstInvocations[last][0]
        before_last_step = last_step - self.res

        aux = []
        for inv in self.lstInvocations:
            if inv[0] > before_last_step and inv[0] <= last_step:
                aux.append(inv[1])

        return float(sum(aux))/float(len(aux)) if len(aux) > 0 else 0  # Success/Fail+Success

    def cost (self) : return 1
    
    def frequency(self): return 1
    
    def success(self, instant) :
        if(len(self.lstInvocations) == 0) : self.lstInvocations = [[instant,1]]
        else : self.lstInvocations.append([instant,1])

    def fail(self,instant) : 
        if(len(self.lstInvocations) == 0) : self.lstInvocations = [[instant,0]]
        else : self.lstInvocations.append([instant,0])
    
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

if __name__ == "__main__":
    analyzer = Analyzer(len(sys.argv), sys.argv)
    analyzer.run()
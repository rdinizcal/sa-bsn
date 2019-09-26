import time
import random
import math

s = 10 ** 9
ms = 10 ** 6
us = 10 ** 3

def main():
    #active_components = ["g4t1", "g3t1_2"] 
    g4t1_freq = 1
    g3t1_2_freq = 50

    now = time.time_ns()
    
    status_file = open("./resource/generated_logs/status_"+str(now)+".log", 'w+')
    event_file = open("./resource/generated_logs/event_"+str(now)+".log", 'w+')
    adaptation_file = open("./resource/generated_logs/adaptation_"+str(now)+".log", 'w+')
    uncertainty_file = open("./resource/generated_logs/uncertainty_"+str(now)+".log", 'w+')

    status_file.write("\n")
    event_file.write("\n")
    adaptation_file.write("\n")
    uncertainty_file.write("\n")

    #initialize components
    instant = 2 * (s)
    logical_clock = 1
    event_file.write("Event,"+str(logical_clock)+","+str(int(instant))+",/g4t1,,activate"+"\n")
    instant += 0.2 * (s)
    logical_clock += 1
    status_file.write("Status,"+str(logical_clock)+","+str(int(instant))+",/g4t1,,init"+"\n")
    instant += 0.2 * (s)
    logical_clock += 1
    event_file.write("Event,"+str(logical_clock)+","+str(int(instant))+",/g3t1_2,,activate"+"\n")
    instant += 0.2 * (s)
    logical_clock += 1
    status_file.write("Status,"+str(logical_clock)+","+str(int(instant))+",/g3t1_2,,init"+"\n")
    logical_clock += 1

    g3t1_2_flag = False
    g4t1_flag = False
    g3t1_2_counter = 0

    R_curr = 1
    R_ref = 1
    Kp = 5
    x=1

    while(instant < 500*(s)):
        instant += ms

        #sum 1 millisecond (in nanoseconds -> 10^⁻3 * 10^9) until achieves one of the desired frequencies
        while(instant % (1/g3t1_2_freq * (s))!= 0 and instant % (1/g4t1_freq * (s)) != 0): 
            instant += ms
            #print("["+str(int(instant))+"]")

        g3t1_2_flag = True if instant % (1/g3t1_2_freq * (s)) == 0 else False
        g4t1_flag = True if instant % (1/g4t1_freq * (s)) == 0 else False

        if(instant < 30*(s)):
            R_curr = 1
        elif(instant == 30*(s)):
            R_curr = 0.6
        elif(instant > 100*(s)):
            g3t1_2_counter += 1
            if(g3t1_2_counter == 10):
                error = R_ref - R_curr
                x = math.ceil(Kp * error) if error > 0 else math.ceil((-1) * Kp * error)
                R_curr = 1-((1-R_curr) ** x)
                g3t1_2_counter = 0
        #   print("["+str(int(instant))+"] R_curr = " + str(R_curr))

        if (g3t1_2_flag) :
            if(random.randint(1,101) < R_curr*100) : status_file.write("Status,"+str(logical_clock)+","+str(int(instant))+",/g3t1_2,,success"+"\n")
            else : status_file.write("Status,"+str(logical_clock)+","+str(int(instant))+",/g3t1_2,,fail"+"\n")

        if (g4t1_flag) : 
            status_file.write("Status,"+str(logical_clock)+","+str(int(instant))+",/g4t1,,success"+"\n")

        logical_clock+=1


    event_file.write("Event,"+str(logical_clock)+","+str(int(instant))+",/g4t1,,deactivate"+"\n")
    instant += 0.2 * (s)
    logical_clock += 1
    event_file.write("Event,"+str(logical_clock)+","+str(int(instant))+",/g3t1_2,,deactivate"+"\n")

if __name__ == "__main__":
    main()
from scipy import stats
import csv 

################ load data ################
with open("../data/summary.csv", newline='') as file:
    s_csv = csv.reader(file, delimiter=',')
    summary = list(s_csv)
    del summary[0] # header1
    del summary[0] # header2
    last = len(summary)-1
    del summary[last] # stdev
    last = len(summary)-1
    del summary[last] # avg

    formula = dict()
    expected_behavior = dict()
    
    for reg in summary:
        if not ("convergence_point" in formula):  formula["convergence_point"] = [float(reg[1])] 
        else : formula["convergence_point"].append(float(reg[1]))

        if not ("settling_time" in formula):  formula["settling_time"] = [float(reg[2])] 
        else : formula["settling_time"].append(float(reg[2])) 

        if not ("overshoot" in formula):  formula["overshoot"] = [float(reg[3])] 
        else : formula["overshoot"].append(float(reg[3])) 

        if not ("steady-state_error" in formula):  formula["steady-state_error"] = [float(reg[4])] 
        else : formula["steady-state_error"].append(float(reg[4]))

        if not ("convergence_point" in expected_behavior):  expected_behavior["convergence_point"] = [float(reg[5])] 
        else : expected_behavior["convergence_point"].append(float(reg[5]))

        if not ("settling_time" in expected_behavior):  expected_behavior["settling_time"] = [float(reg[6])] 
        else : expected_behavior["settling_time"].append(float(reg[6])) 

        if not ("overshoot" in expected_behavior):  expected_behavior["overshoot"] = [float(reg[7])] 
        else : expected_behavior["overshoot"].append(float(reg[7])) 

        if not ("steady-state_error" in expected_behavior):  expected_behavior["steady-state_error"] = [float(reg[8])] 
        else : expected_behavior["steady-state_error"].append(float(reg[8])) 
    
    for tag in formula:
        y = formula[tag]
        x = expected_behavior[tag]
        
        #print("%s p-value = %s" % (tag.replace("_", " "), stats.ttest_ind(x, y)))
        print("%s p-value = %f" % (tag.replace("_", " "), stats.wilcoxon(x, y, zero_method='wilcox', correction=True)[1]) )

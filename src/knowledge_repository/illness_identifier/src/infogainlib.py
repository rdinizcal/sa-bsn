#!/usr/bin/env python

import csv
import math
'''------------------------------ Entropy ---------------------------'''

# Calculates the entropy of the given data set for the target attribute.
def entropy(data, target_attr):
 
    val_freq = {}
    data_entropy = 0.0
    
    # Calculate the frequency of each of the values in the target attr
    for record in data:
        if (record[target_attr] in val_freq):
            val_freq[record[target_attr]] += 1.0
        else:
            val_freq[record[target_attr]]  = 1.0
 
    # Calculate the entropy of the data for the target attribute
    for freq in val_freq.values():
        data_entropy += (-freq/len(data)) * math.log(freq/len(data), 2) 
 
    return data_entropy

'''------------------------------ Information Gain ---------------------------'''

# Calculates the information gain (reduction in entropy) that
#   would result by splittingthe data on the chosen attribute (attr).

def gain(data, attr, target_attr):
 
    val_freq = {}
    subset_entropy = 0.0
 
    # Calculate the frequency of each of the values in the target attribute
    for record in data:
        if (record[attr] in val_freq):
            val_freq[record[attr]] += 1.0
        else:
            val_freq[record[attr]]  = 1.0
 
    # Calculate the sum of the entropy for each subset of records weighted by their probability of occuring in the training set.
    for val in val_freq.keys():
        val_prob = val_freq[val] / sum(val_freq.values())
        data_subset = [record for record in data if record[attr] == val]
        subset_entropy += val_prob * entropy(data_subset, target_attr)
 
    # Subtract the entropy of the chosen attribute from the entropy of the whole data set with respect to the target attribute (and return it)
    return (entropy(data, target_attr) - subset_entropy)

'''------------------------------ Main ---------------------------'''

if __name__ == '__main__':

    gains = {}
    
    with open("template.csv") as f:
        read = csv.reader(f)
        features = next(read)
    i = 0
    with open("template.csv") as f:
        reader = csv.DictReader(f)
        data = [r for r in reader]
    
    print(type(data))
    ent = entropy(data, features[-1])
##    print("\n\n--------------------------------------------------------------")
    #print("Shannon's Entropy of class '"+features[-1]+"': ",ent)
##    print("--------------------------------------------------------------\n")
    
    for item in range(0, len(features)-1):
##        gain(data, features[item], features[-1])
        #print(features[item]+"'s information gain:",gain(data, features[item], features[-1]))
        item+=1
##    print("\n------------------------------------------------------------\n")
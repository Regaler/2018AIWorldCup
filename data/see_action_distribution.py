import pickle
import numpy as np
import sys

filename = sys.argv[1]

if 'label' not in filename:
	print("Error: you should input a label file.")
	exit()

with open(filename,'rb') as f:
    mydata = pickle.load(f)

#print(mydata)
arr = np.array(mydata)
result = np.sum(arr,axis=0)
print(result)

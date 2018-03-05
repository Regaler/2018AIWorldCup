import pickle
import numpy as np
import sys

if sys.argv[1] == '--train':
    filename = './train/label.pkl'
elif sys.argv[1] == '--test':
    filename = './test/label.pkl'

with open(filename,'rb') as f:
    mydata = pickle.load(f)

#print(mydata)
arr = np.array(mydata)
result = np.sum(arr,axis=0)
print(result)

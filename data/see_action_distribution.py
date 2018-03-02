import pickle
import numpy as np

with open('./label0.pkl','rb') as f:
    mydata = pickle.load(f)

#print(mydata)
arr = np.array(mydata)
result = np.sum(arr,axis=0)
print(result)
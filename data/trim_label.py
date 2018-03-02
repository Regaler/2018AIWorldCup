import pickle
import numpy as np

with open('./label0_test.pkl','rb') as f:
    mydata = pickle.load(f)

# print(mydata)
arr = np.array(mydata)
arr = arr[:,[1,2,3,4,5,6,7,8,10,16,17,18]]
print(arr[:10])

with open('./label0_test_trimed.pkl','wb') as f:
	pickle.dump(arr, f)

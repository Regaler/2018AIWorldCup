import pickle
import sys

filename = sys.argv[1]

with open(filename,'rb') as f:
    mydata = pickle.load(f)

print(mydata)
print("# of data: " + str(len(mydata)))
print(len(mydata[0]))

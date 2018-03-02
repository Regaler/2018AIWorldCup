import pickle
import sys

filename = sys.argv[1]

with open(filename,'rb') as f:
    mydata = pickle.load(f)

print("\nmydata[0] is: ")
print(mydata[0])
print("# of data: " + str(len(mydata)))
try:
	print("len(mydata[0]): " + str(len(mydata[0])))
	print("len(mydata[1]): " + str(len(mydata[1])))
	print("len(mydata[2]): " + str(len(mydata[2])))
except:
	print("Error: Length of data must be larger than 3.")
print("\n")
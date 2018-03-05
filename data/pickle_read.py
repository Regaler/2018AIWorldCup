import pickle
import sys

filename = sys.argv[1]
mydata = list()

def read_from_pickle(path):
    with open(path, 'rb') as file:
        try:
            while True:
                data = pickle.load(file)
                if data == "":
                	return mydata
                else:
                	mydata.append(data)
        except EOFError:
            return mydata
"""
with open(filename,'rb') as f:
    while True:
        data = pickle.load(f)
        if data == None:
            break
        mydata.append(data)
"""
D = read_from_pickle(filename)
#print(D)
print("# of data: " + str(len(D)))
print("data[0]: " + str(D[0]))

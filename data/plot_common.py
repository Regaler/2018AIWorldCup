import matplotlib.pyplot as plt
import numpy as np
import sys

def moving_average(values, window):
	weights = np.repeat(1.0, window)/window
	sma = np.convolve(values, weights, 'valid')
	return sma

qs = []
filename = sys.argv[1]

with open(filename,'r') as f:
	while True:
		line = f.readline()
		if line == '':
			break
		q = float(line)
		qs.append(q)


qs = np.array(qs)



qs = moving_average(qs,30)
x = range(len(qs))

#time = 1000
x = x[:]
qs = qs[:]

plt.plot(x,qs,'r')
plt.show()

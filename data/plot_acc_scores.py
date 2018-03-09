import matplotlib.pyplot as plt
import numpy as np

wins = []
losses = []

with open("./scores.txt",'r') as f:
	while True:
		line = f.readline()
		if line == '':
			break
		print(line)
		win, loss = line.split(', ')
		win = int(win)
		loss = int(loss)
		wins.append(win)
		losses.append(loss)

#print(wins)
#print(losses)
x = range(len(wins))
wins = np.array(wins)
losses = np.array(losses)
wins = np.cumsum(wins)
losses = np.cumsum(losses)
print(wins)

start = 0
time = -1
x = x[start:time]
wins = wins[start:time]
losses = losses[start:time]

plt.plot(x,wins,'ro',x,losses,'b^')
plt.show()

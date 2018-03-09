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

print(wins)
print(losses)
x = range(len(wins))
wins = np.array(wins)
losses = np.array(losses)
#print(wins)

W = []
L = []
S_W = 0
S_L = 0
for i in range(len(wins)):
	if i % 14 == 13:
		S_W += wins[i]
		S_L += losses[i]
		W.append(S_W)
		L.append(S_L)
		S_W = 0
		S_L = 0

print("W is: " + str(W))


start = 0
#time = -1
x = x[start:]
"""
wins = wins[start:time]
losses = losses[start:time]

plt.plot(x,wins,'ro',x,losses,'b^')
plt.show()
"""
W = W[start:]
L = L[start:]
x = range(len(W))
plt.plot(x,W,'ro',x,L,'b^')
plt.show()

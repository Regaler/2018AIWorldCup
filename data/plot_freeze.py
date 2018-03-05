import matplotlib.pyplot as plt


wins = []
losses = []

with open("../freeze/scores.txt",'r') as f:
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
plt.plot(x,wins,'ro',x,losses,'b^')
plt.show()

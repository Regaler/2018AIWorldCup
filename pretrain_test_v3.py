import argparse
import skimage as skimage
from skimage import io
from skimage import transform, color, exposure
from skimage.transform import rotate
from skimage.viewer import ImageViewer
import sys
from scipy import misc

import numpy as np
from collections import deque
import PIL
from PIL import Image
import random
import pickle

import json
#from keras import initializers
#from keras.initializers import normal, identity
from keras.models import model_from_json
from keras.models import Sequential
from keras.layers.core import Dense, Dropout, Activation, Flatten
from keras.layers.convolutional import Convolution2D, MaxPooling2D
from keras.optimizers import SGD , Adam
from keras import regularizers
#import keras.backend
import tensorflow as tf
from keras import losses
import keras
import numpy as np
import sys

"""
$python3 pretrain_test_v3.py --train
$python3 pretrain_test_v3.py --test
"""

def read_from_pickle(path):
	mydata = []
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

# Neural network
def buildmodel():
	model = Sequential()
	model.add(Dense(64, input_dim = state_size, activation = 'relu'))
	model.add(Dense(128, activation = 'relu'))
	model.add(Dense(128, activation = 'relu'))
	model.add(Dropout(0.5))
	model.add(Dense(label_size, activation = 'softmax', kernel_regularizer = regularizers.l2(0.01))) #activity_regularizer=regularizers.l1(0.01)))
	model.compile(loss='mse', optimizer = Adam(lr=LEARNING_RATE))
	return model

if __name__ == 	"__main__":
	flag = sys.argv[1]
	
	#Convert image into Black and white
	if flag == "--train":
		STATE = './data/train/COORDINATE.pkl'
		LABEL = './data/train/label.pkl'
	elif flag == "--test":
		STATE = './data/test/COORDINATE.pkl'
		LABEL = './data/test/label.pkl'
	else:
		print("Error: No such flag. You can only input --train or --test")

	LEARNING_RATE = 1e-5
	state_size = 34
	label_size = 10
	batch_size = 32
	num_of_data = 0
	states = []
	labels = []
	loss = 0
	prediction_success_ratio = 0

	# <1> make neural net
	model = buildmodel()
	model.load_weights("./save/weights_FC_ddpg.h5")

	# <2> Restore data and label
	labels = read_from_pickle(LABEL)
	states = read_from_pickle(STATE)
	num_of_data = len(states)

	X = np.zeros((batch_size, state_size))
	Y = np.zeros((batch_size, label_size))
	cnt_batch = 0

	# Make batch
	for cnt_batch in range(batch_size):
		i = int(num_of_data*random.random())
		X[cnt_batch] = np.array(states[i])
		Y[cnt_batch] = np.array(labels[i])

	# Predict and calculate loss
	Y_predict = model.predict(X)
	loss = ((Y_predict - Y) ** 2).mean()
	print("The loss: " + str(loss))

	"""
	# <3> Run 5 times for no reason
	for epoch in range(100):
		cnt = 0
		cnt_total = 0
		cnt_batch = 0
		X = np.zeros((batch_size,state_size))
		Y = np.zeros((batch_size,label_size))
		#print(epoch)

		# Make batch of size 32
		for cnt_batch in range(batch_size):
			i = int(num_of_data*random.random())
			X[cnt_batch] = states[i]
			Y[cnt_batch] = labels[i]

		# Predict and calculate loss
		Y_predict = model.predict(X)
		loss_temp = ((Y_predict - Y) ** 2).mean()
		loss += loss_temp

		# Write the result to file
		with open("./data/test_result.txt",'a') as ff:
			ff.write("loss_temp: " + str(loss_temp) + "\n")

	loss /= 100
	print("\naverage loss : " + str(loss))
	"""
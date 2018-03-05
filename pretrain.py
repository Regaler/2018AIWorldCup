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
import tensorflow as tf
from keras import losses
from keras import regularizers
import keras
import numpy as np

"""
$python3 pretrain.py 0
"""

def buildmodel():
	model = Sequential()
	model.add(Dense(64, input_dim = state_size, activation = 'relu'))
	model.add(Dense(128, activation = 'relu'))
	model.add(Dense(128, activation = 'relu'))
	model.add(Dropout(0.5))
	model.add(Dense(label_size, activation = 'softmax', kernel_regularizer = regularizers.l2(0.01))) #activity_regularizer=regularizers.l1(0.01)))
	model.compile(loss='mse', optimizer = Adam(lr=LEARNING_RATE))
	return model

if __name__ == '__main__':
	# parse
	flag = sys.argv[1]
	if flag == '0':
		label_size = 12
	elif flag == '1':
		label_size = 13
	elif flag == '2':
		label_size = 13
	elif flag == '3':
		label_size = 14
	else:
		print("Error: The argument must be one of 0, 1, 2, or 3")
		exit()

	# Iniaizliation
	LEARNING_RATE = 5*1e-5
	LABEL = './data/train/label' + str(flag) + '.pkl'
	COORDINATION = './data/train/COORDINATION' + str(flag) + '.pkl'
	save_file = './save/weights_FC' + str(flag) + '.h5'
	BATCH = 64
	state_size = 34
	num_of_data = 0

	labels = []
	states = []

	# Restore data and label
	with open(LABEL,'rb') as g, open(COORDINATION,'rb') as h:
		labels = pickle.load(g)[1:]
		states = pickle.load(h)[1:]
		num_of_data = len(states)

	# make neural net
	model = buildmodel()
	EPOCH = num_of_data*200
	print("num_of_data: " + str(num_of_data))
	print("len(states): " + str(len(states)))

	# Run 200 timess
	for epoch in range(EPOCH):
		X_temp = np.zeros((BATCH, state_size))
		Y_temp = np.zeros((BATCH, label_size))
		cnt_batch = 0
		print(epoch)

		# Make batch
		for cnt_batch in range(BATCH):
			i = int(num_of_data*random.random())
			X_temp[cnt_batch] = np.array(states[i])
			Y_temp[cnt_batch] = np.array(labels[i])

		# forward and backward pass
		model.fit(X_temp, Y_temp, epochs=1)

		# Print loss at the end
		if epoch == EPOCH - 1:
			Y_pred = model.predict(X_temp)
			print("true: " + str(Y_temp))
			print("pred: " + str(Y_pred))

		# Save weights
		if epoch % 1000 == 0 and epoch > 1:
			model.save_weights(save_file, overwrite=True)
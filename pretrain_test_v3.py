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
$python3 pretrain_test_v3.py --train 0
"""

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
	num_flag = sys.argv[2]
	label_size = 0
	
	#Convert image into Black and white
	if flag == "--train":
		STATE = './data/train/COORDINATION' + num_flag + '.pkl'
		LABEL = './data/train/label' + num_flag + '.pkl'
	elif flag == "--test":
		STATE = './data/test/COORDINATION' + num_flag + '.pkl'
		LABEL = './data/test/label' + num_flag + '.pkl'
	else:
		print("Error: No such flag. You can only input --train or --test")

	if num_flag == '0':
		label_size = 5
	elif num_flag == '1':
		label_size = 5
	elif num_flag == '2':
		label_size = 5
	elif num_flag == '3':
		label_size = 5
	else:
		print("Error: No such robot. You should give 0, 1, 2, or 3")

	LEARNING_RATE = 1e-5
	state_size = 47
	batch_size = 32
	num_of_data = 0
	states = []
	labels = []
	loss = 0
	prediction_success_ratio = 0

	# <1> make neural net
	model = buildmodel()
	model.load_weights("./save/weights_FC" + num_flag + ".h5")

	# <2> Restore data and label
	with open(STATE,'rb') as f, open(LABEL,'rb') as g:
		states = pickle.load(f)
		labels = pickle.load(g)
	states = states[2:]
	labels = labels[2:]
	num_of_data = len(labels)

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
		#print("true: " + str(Y))
		#print("pred: " + str(Y_predict))
		#print("loss :" + str(loss_temp))

		for i in range(32):
			if np.argmax(Y[i])==np.argmax(Y_predict[i]):
				cnt+=1
			cnt_total+=1

		# Write the result to file
		with open("./data/test_result.txt",'a') as ff:
			ff.write("**************************\n")
			for j in range(32):
				ff.write("action number: " + str(np.argmax(Y[j])) + ", predicted action number: " + str(np.argmax(Y_predict[j])) +"\n")
			ff.write("Prediction success ratio: " + str(cnt/cnt_total) + "\n")
		prediction_success_ratio += cnt/cnt_total

	loss /= 100
	print("\naverage loss : " + str(loss))
	prediction_success_ratio /= 100
	print("average prediction_success_ratio : " + str(prediction_success_ratio) + "\n")

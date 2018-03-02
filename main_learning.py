#!/usr/bin/python3

from twisted.internet import reactor
from twisted.internet.defer import inlineCallbacks

from autobahn.wamp.serializer import MsgPackSerializer
from autobahn.wamp.types import ComponentConfig
from autobahn.twisted.wamp import ApplicationSession, ApplicationRunner

import argparse
import random
import sys
import pickle

from calculator import *
from data_processor import *
from strategy_v17 import *
from motor import *
import math
from PIL import Image
import numpy as np
from collections import deque

import os
import base64

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
import keras

#reset_reason
NONE = 0
GAME_START = 1
SCORE_MYTEAM = 2
SCORE_OPPONENT = 3
GAME_END = 4
DEADLOCK = 5

#coordinates
MY_TEAM = 0
OP_TEAM = 1
BALL = 2
X = 0
Y = 1
TH = 2

def ActionNumInterpreter(id, strategy, y):
    our_posture = strategy.data_proc.get_my_team_postures()
    my_posture = our_posture[id]
    cur_ball = strategy.data_proc.get_cur_ball_position()
    cur_trans = strategy.data_proc.get_cur_ball_transition()
    theta_to_ball = strategy.cal.compute_theta_to_target(my_posture, cur_ball)
    static_theta = strategy.cal.compute_static_theta(my_posture, cur_ball)
    dist_to_ball = strategy.cal.get_distance(my_posture,cur_ball)

    my_x = my_posture[0]
    my_y = my_posture[1]
    th = my_posture[2]
    bx = cur_ball[0]
    by = cur_ball[1]

    backup_x = 1.5  
    backup_y = GOAL_WIDTH-0.2

    wait_x = -1.85
    if (by >= GOAL_WIDTH-0.1):
        wait_y = GOAL_WIDTH-0.1
    elif (by <= -GOAL_WIDTH+0.1):
        wait_y = -GOAL_WIDTH+0.1
    else: 
        wait_y = by

    i = np.argmax(y)

    if id == 0: #------------------------------
        if i == 0:
            goal_pstn = [1.9, 0]
            tar_posture = strategy.cal.compute_desired_posture(cur_ball, cur_trans, goal_pstn)
            tar_posture = [tar_posture[0]-0.05,tar_posture[1]-0.05,0]
            wheel_velos = strategy.motors[id].move_to_target(my_posture, tar_posture, damping=0)
        elif i == 1:
            wheel_velos = strategy.motors[id].move_to_target(my_posture, [my_x,my_y-1])
        elif i == 2:
            wheel_velos = strategy.motors[id].move_to_target(my_posture, [my_x,my_y+1])
        elif i == 3:
            goal_pstn = [1.9, 0]
            tar_posture = strategy.cal.compute_desired_posture(cur_ball, cur_trans, goal_pstn)
            tar_posture = [tar_posture[0]-0.08,tar_posture[1]-0.08,0]
            wheel_velos = strategy.motors[id].move_to_target(my_posture, tar_posture, damping=0)
        elif i == 4:
            wheel_velos = strategy.motors[id].move_to_target(my_posture, cur_ball, damping=0)
        elif i == 5:
            wheel_velos = [-1.5, -1.8]
        elif i == 6:
            wheel_velos = [-1.8, -1.5]
        elif i == 7:
            target = [cur_ball[0] + cur_trans[0], cur_ball[1] + cur_trans[1]]
            wheel_velos = strategy.motors[id].move_to_target(my_posture, target, damping=0.2)
        elif i == 8:
            NUM_OF_PREDICTED_FRAMES = 2
            target = [cur_ball[0] + cur_trans[0]*NUM_OF_PREDICTED_FRAMES, cur_ball[1] + cur_trans[1]*NUM_OF_PREDICTED_FRAMES]
            wheel_velos = strategy.motors[id].move_to_target(my_posture, target, damping=0)
        elif i == 9:
            wheel_velos = strategy.motors[id].spin_to_theta(th,strategy.cal.d2r(-90))
        elif i == 10:
            wheel_velos = strategy.motors[id].three_phase_move_to_target(my_posture, [-1.85, -GOAL_WIDTH-0.2, math.pi/2])
        elif i == 11:
            wheel_velos = strategy.motors[id].three_phase_move_to_target(my_posture, [-1.2, GOAL_WIDTH+0.2, static_theta])
        else:
            pass    
        return wheel_velos
    elif id == 1: #------------------------------
        if i == 0:
            goal_pstn = [1.9, 0]
            tar_posture = strategy.cal.compute_desired_posture(cur_ball, cur_trans, goal_pstn)
            tar_posture = [tar_posture[0]-0.05,tar_posture[1]+0.05,0]
            wheel_velos = strategy.motors[id].move_to_target(my_posture, tar_posture, damping=0)

        elif i == 1:
            wheel_velos = strategy.motors[id].move_to_target(my_posture, [my_x,my_y-1])

        elif i == 2:
            wheel_velos = strategy.motors[id].move_to_target(my_posture, [my_x,my_y+1])

        elif i == 3:
            goal_pstn = [1.9, 0]
            tar_posture = strategy.cal.compute_desired_posture(cur_ball, cur_trans, goal_pstn)
            tar_posture = [tar_posture[0]-0.08,tar_posture[1]-0.08,0]
            wheel_velos = strategy.motors[id].move_to_target(my_posture, tar_posture, damping=0)

        elif i == 4:
             wheel_velos = strategy.motors[id].move_to_target(my_posture, cur_ball, damping=0)

        elif i == 5:
            wheel_velos = [-1.5, -1.8]

        elif i == 6:
            wheel_velos = [-1.8, -1.5]

        elif i == 7:
            target = [cur_ball[0] + cur_trans[0], cur_ball[1] + cur_trans[1]]
            wheel_velos = strategy.motors[id].move_to_target(my_posture, target, damping=0.2)

        elif i == 8:
            NUM_OF_PREDICTED_FRAMES = 2
            target = [cur_ball[0] + cur_trans[0]*NUM_OF_PREDICTED_FRAMES, cur_ball[1] + cur_trans[1]*NUM_OF_PREDICTED_FRAMES]
            wheel_velos = strategy.motors[id].move_to_target(my_posture, target, damping=0)

        elif i == 9:
            wheel_velos = [1.8, 1.8]

        elif i == 10:
            wheel_velos = [0, 0]

        elif i == 11:
            wheel_velos = strategy.motors[id].spin_to_theta(th, strategy.cal.d2r(90))

        elif i == 12:
            wheel_velos = strategy.motors[id].three_phase_move_to_target(my_posture, [-1.85, GOAL_WIDTH+0.2, math.pi/2])

        elif i == 13:
            wheel_velos = strategy.motors[id].three_phase_move_to_target(my_posture, [-1.2, -GOAL_WIDTH-0.2, static_theta])
        else:
            pass
        #printWrapper("Exit ActionNumInterpreter")
        return wheel_velos
    elif id == 2: #------------------------------
        if i == 0:
            wheel_velos = strategy.motors[id].move_to_target(my_posture, [my_x,my_y-1])

        elif i == 1:
            wheel_velos = strategy.motors[id].move_to_target(my_posture, [my_x,my_y+1])

        elif i == 2:
            goal_pstn = [1.9, 0]
            tar_posture = strategy.cal.compute_desired_posture(cur_ball, cur_trans, goal_pstn)
            tar_posture = [tar_posture[0]-0.08,tar_posture[1]-0.08,0]
            wheel_velos = strategy.motors[id].move_to_target(my_posture, tar_posture, damping=0)

        elif i == 3:
            wheel_velos = [-1.8, -1.5]

        elif i == 4:
            target = [cur_ball[0] + cur_trans[0], cur_ball[1] + cur_trans[1]]
            wheel_velos = strategy.motors[id].move_to_target(my_posture, target, damping=0.2)

        elif i == 5:
            NUM_OF_PREDICTED_FRAMES = 2
            target = [cur_ball[0] + cur_trans[0]*NUM_OF_PREDICTED_FRAMES, cur_ball[1] + cur_trans[1]*NUM_OF_PREDICTED_FRAMES]
            wheel_velos = strategy.motors[id].move_to_target(my_posture, target, damping=0)

        elif i == 6:
            wheel_velos = strategy.motors[id].move_to_target(my_posture, [wait_x-0.02, wait_y], damping=0)

        elif i == 7:
            wheel_velos = strategy.motors[id].three_phase_move_to_target(my_posture, [-1.0, min(0.0, cur_ball[1]), static_theta])

        elif i == 8:
            target = [cur_ball[0] + cur_trans[0], cur_ball[1] + cur_trans[1]]
            wheel_velos = strategy.motors[id].move_to_target(my_posture, target, damping=0.1)

        elif i == 9:
            NUM_OF_PREDICTED_FRAMES = 3
            target = [cur_ball[0] + cur_trans[0]*NUM_OF_PREDICTED_FRAMES, cur_ball[1] + cur_trans[1]*NUM_OF_PREDICTED_FRAMES]
            wheel_velos = strategy.motors[id].move_to_target(my_posture, target, damping=0.2)

        elif i == 10:
            wheel_velos = [-1.8, -1.8]

        elif i == 11:
            wheel_velos = strategy.motors[id].three_phase_move_to_target(my_posture, [backup_x, -backup_y, 0])

        elif i == 12:
            wheel_velos = strategy.motors[id].move_to_target(my_posture, [backup_x,0], damping=0)
        else:
            pass
        #printWrapper("Exit ActionNumInterpreter")
        return wheel_velos
    elif id == 3: #------------------------------
        if i == 0:
            wheel_velos = strategy.motors[id].move_to_target(my_posture, [my_x,my_y-1])

        elif i == 1:
            wheel_velos = strategy.motors[id].move_to_target(my_posture, [my_x,my_y+1])

        elif i == 2:
            goal_pstn = [1.9, 0]
            tar_posture = strategy.cal.compute_desired_posture(cur_ball, cur_trans, goal_pstn)
            tar_posture = [tar_posture[0]-0.08,tar_posture[1]-0.08,0]
            wheel_velos = strategy.motors[id].move_to_target(my_posture, tar_posture, damping=0)

        elif i == 3:
            wheel_velos = [-1.5, -1.8]

        elif i == 4:
            wheel_velos = [-1.8, -1.5]

        elif i == 5:
            target = [cur_ball[0] + cur_trans[0], cur_ball[1] + cur_trans[1]]
            wheel_velos = strategy.motors[id].move_to_target(my_posture, target, damping=0.2)

        elif i == 6:
            NUM_OF_PREDICTED_FRAMES = 2
            target = [cur_ball[0] + cur_trans[0]*NUM_OF_PREDICTED_FRAMES, cur_ball[1] + cur_trans[1]*NUM_OF_PREDICTED_FRAMES]
            wheel_velos = strategy.motors[id].move_to_target(my_posture, target, damping=0)

        elif i == 7:
            wheel_velos = strategy.motors[id].spin_to_theta(th, strategy.cal.d2r(90))

        elif i == 8:
            wheel_velos = strategy.motors[id].move_to_target(my_posture, [wait_x-0.02, wait_y], damping=0)

        elif i == 9:
            wheel_velos = strategy.motors[id].move_to_target(my_posture, [min(wait_x,bx),by], damping=0)

        elif i == 10:
            target = [cur_ball[0] + cur_trans[0], cur_ball[1] + cur_trans[1]]
            wheel_velos = strategy.motors[id].move_to_target(my_posture, [target[0]+0.2,target[1]-0.05], damping=0)

        elif i == 11:
            target = [cur_ball[0] + cur_trans[0], cur_ball[1] + cur_trans[1]]
            wheel_velos = strategy.motors[id].move_to_target(my_posture, [min(wait_x,target[0]-0.05),target[1]-0.05], damping=0)

        elif i == 12:
            wheel_velos = strategy.motors[id].three_phase_move_to_target(my_posture, [-1.0, min(0.0, cur_ball[1]), static_theta])

        elif i == 13:
            NUM_OF_PREDICTED_FRAMES = 3
            target = [cur_ball[0] + cur_trans[0]*NUM_OF_PREDICTED_FRAMES, cur_ball[1] + cur_trans[1]*NUM_OF_PREDICTED_FRAMES]
            wheel_velos = strategy.motors[id].move_to_target(my_posture, target, damping=0.2)

        elif i == 14:
            wheel_velos = [-1.8, -1.8]

        elif i == 15:
            wheel_velos = strategy.motors[id].three_phase_move_to_target(my_posture, [backup_x, +backup_y, 0])

        elif i == 16:
            wheel_velos = strategy.motors[id].move_to_target(my_posture, [backup_x,0], damping=0)
        else:
            pass
        #printWrapper("Exit ActionNumInterpreter")
        return wheel_velos
    else:
        assert(False), "id2info: No such id"

def ActionNumInterpreter3(strategy, y, id):
    #printWrapper("Called ActionNumInterpreter")
    our_posture = strategy.data_proc.get_my_team_postures()
    my_posture = our_posture[id]
    cur_ball = strategy.data_proc.get_cur_ball_position()
    cur_trans = strategy.data_proc.get_cur_ball_transition()
    theta_to_ball = strategy.cal.compute_theta_to_target(my_posture, cur_ball)
    static_theta = strategy.cal.compute_static_theta(my_posture, cur_ball)
    dist_to_ball = strategy.cal.get_distance(my_posture,cur_ball)

    my_x = my_posture[0]
    my_y = my_posture[1]
    th = my_posture[2]
    bx = cur_ball[0]
    by = cur_ball[1]

    backup_x = 1.5  
    backup_y = GOAL_WIDTH-0.2

    wait_x = -1.85
    if (by >= GOAL_WIDTH-0.1):
        wait_y = GOAL_WIDTH-0.1
    elif (by <= -GOAL_WIDTH+0.1):
        wait_y = -GOAL_WIDTH+0.1
    else: 
        wait_y = by

    i = np.argmax(y)

    

def printWrapper(msg):
    print(msg)
    sys.__stdout__.flush() # stdout is redirected to somewhere else. flush __stdout__ directly.

def hot_encode(mylist):
    result = []
    for i in range(4):
        temp = np.zeros(32)
        temp[mylist[i]] += 1
        temp = [int(x) for x in temp]
        result.append(list(temp))
    return result
        
class Received_Image(object):
    def __init__(self, resolution, colorChannels):
        #printWrapper("1111")
        self.resolution = resolution
        self.colorChannels = colorChannels
        # need to initialize the matrix at timestep 0
        self.ImageBuffer = np.zeros((resolution[1], resolution[0], colorChannels)) # rows, columns, colorchannels
        #printWrapper("2222")
    def update_image(self, received_parts):
        self.received_parts = received_parts
        for i in range(0,len(received_parts)):
           dec_msg = base64.b64decode(self.received_parts[i].b64, '-_') # decode the base64 message
           np_msg = np.fromstring(dec_msg, dtype=np.uint8) # convert byte array to numpy array
           reshaped_msg = np_msg.reshape((self.received_parts[i].height, self.received_parts[i].width, 3))
           for j in range(0, self.received_parts[i].height): # y axis
               for k in range(0, self.received_parts[i].width): # x axis
                   self.ImageBuffer[j+self.received_parts[i].y, k+self.received_parts[i].x, 0] = reshaped_msg[j, k, 0] # blue channel
                   self.ImageBuffer[j+self.received_parts[i].y, k+self.received_parts[i].x, 1] = reshaped_msg[j, k, 1] # green channel     
                   self.ImageBuffer[j+self.received_parts[i].y, k+self.received_parts[i].x, 2] = reshaped_msg[j, k, 2] # red channel            

class SubImage(object):
    def __init__(self, x, y, width, height, b64):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.b64 = b64

class Frame(object):
    def __init__(self):
        self.time = None
        self.score = None
        self.reset_reason = None
        self.subimages = None
        self.coordinates = None

class Component(ApplicationSession):
    """
    AI Base + Random Walk
    """ 

    def __init__(self, config):
        ApplicationSession.__init__(self, config)
        self.colorChannels = 3 # nf
        self.end_of_frame = False
        self._frame = 0

        self.memory0 = deque(maxlen=2000) # Replay Memory D
        self.memory1 = deque(maxlen=2000)
        self.memory2 = deque(maxlen=2000)
        self.memory3 = deque(maxlen=2000)

        self.epsilon = 0.5
        self.final_epsilon = 0.01 # Final epsilon value
        self.dec_epsilon = 0.001 # Decrease rate of epsilon for every generation
        self.gamma = 0.99 # 0.99
        self.batch_size = 32
        self.learning_rate = 1e-5

        self.cnt = 0
        self.state_size = 47
        self.model0 = self.build_model(0)
        self.model0.load_weights("./save/weights_FC0.h5")
        self.model1 = self.build_model(1)
        self.model1.load_weights("./save/weights_FC1.h5")
        self.model2 = self.build_model(2)
        self.model2.load_weights("./save/weights_FC2.h5")
        self.model3 = self.build_model(3)
        self.model3.load_weights("./save/weights_FC3.h5")

        self.done = False
        self.losscounter = 0
        self.average_loss=[]
        self.cumulative_reward = 0
        self.my_team_score = 0
        self.opp_team_score = 0

    def onConnect(self):
        printWrapper("Transport connected")
        self.join(self.config.realm)

    """Some methods"""
    def id2info(self, id):
        if id == 0:
            return 12, self.memory0, self.model0
        elif id == 1:
            return 14, self.memory1, self.model1
        elif id == 2:
            return 13, self.memory2, self.model2
        elif id == 3:
            return 17, self.memory3, self.model3
        else:
            assert(False), "id2info: No such id"

    def build_model(self, id):
        if id == 0:
            label_size = 12
        elif id == 1:
            label_size = 14
        elif id == 2:
            label_size = 13
        elif id == 3:
            label_size = 17
        else:
            assert(False), 'wrong id'

        model = Sequential()
        model.add(Dense(64, input_dim = self.state_size, activation = 'relu'))
        model.add(Dense(128, activation = 'relu'))
        model.add(Dense(128, activation = 'relu'))
        model.add(Dense(label_size, activation = 'softmax'))#, kernel_regularizer = regularizers.l2(0.01))) #activity_regularizer=regularizers.l1(0.01)))
        model.compile(loss='mse', optimizer = Adam(lr=self.learning_rate))
        return model

    def remember(self, id, state_action_reward_triplet):
        label_size, memory, model = self.id2info(id)
        memory.append((state_action_reward_triplet[0], state_action_reward_triplet[1], state_action_reward_triplet[2]))

    def act(self, id, state):
        label_size, memory, model = self.id2info(id)
        if np.random.rand() <= self.epsilon:
            return random.randrange(label_size)
        else:
            #printWrapper("in act function state shape: " + str(state.shape))
            #printWrapper("state: " + str(state))
            act_values = model.predict(state) # act_values is of the form: [[0,1,0,0,0,...,0]]
            return np.argmax(act_values[0])

    def replay(self, id,  batch_size):
        printWrapper("entered replay")
        label_size, memory, model = self.id2info(id)
        #state_set = np.zeros((batch_size,self.img_rows,self.img_cols,3))
        state_set = np.zeros((batch_size, self.state_size))
        target_f_set=np.zeros((batch_size,label_size))
        # make batch
        for i in range(batch_size-1):
            index = np.random.randint(len(memory)-1)
            state = memory[index][0]
            action = memory[index][1]
            reward = memory[index][2]
            next_state = memory[index+1][0]
            # size cast

            target = reward + self.gamma * np.amax(model.predict(next_state)[0])
            target_f = model.predict(state)
            state_set[i] = state
            target_f[0][action] = target
            target_f_set[i] = target_f

        model.fit(state_set, target_f_set, epochs=1, verbose=0)
        self.epsilon = max(self.epsilon - self.dec_epsilon, self.final_epsilon)
        printWrapper("Exit replay")


    def write_loss(self, id, batch_size):
        label_size, memory, model = self.id2info(id)

        state_set = np.zeros((batch_size, self.state_size))
        target_f_set = np.zeros((batch_size, label_size))
        # make batch
        for i in range(batch_size-1):
            index = np.random.randint(len(memory)-1)
            state = memory[index][0]
            action = memory[index][1]
            reward = memory[index][2]
            next_state = memory[index+1][0]

            target = reward + self.gamma * np.amax(model.predict(next_state)[0])
            target_f = model.predict(state)
            state_set[i] = state
            target_f[0][action] = target
            target_f_set[i] = target_f

        Y_pred = model.predict(state_set)
        Y_true = target_f_set
        loss = ((Y_pred - Y_true) ** 2).mean()
        printWrapper("loss: " + str(loss))

        with open("./data/loss_value.txt",'a') as ff, open("./data/q_value.txt",'a') as hh:
            ff.write(str(loss) + "\n")
            hh.write(str(np.max(Y_pred[0])) + "\n")

    def load(self, id, name):
        label_size, memory, model = self.id2info(id)
        model.load_weights(name)

    def save(self, id, name):
        label_size, memory, model = self.id2info(id)
        model.save_weights(name)

    @inlineCallbacks
    def onJoin(self, details):
        printWrapper("session attached")

##############################################################################
        def init_variables(self, info):
            # Here you have the information of the game (virtual init() in random_walk.cpp)
            # List: game_time, goal, number_of_robots, penalty_area, codewords,
            #       robot_size, max_linear_velocity, field, team_info,
            #       {rating, name}, axle_length, resolution, ball_radius
            # self.game_time = info['game_time']
            # self.field = info['field']
            self.max_linear_velocity = info['max_linear_velocity']
            self.resolution = info['resolution']
            self.image = Received_Image(self.resolution, self.colorChannels)
            printWrapper("Initializing variables...")
            self.data_proc = Data_processor(is_debug=False)
            self.strategy = Strategy(self.data_proc, is_debug=False)
            self.path=['./data/robot0','./data/robot1','./data/robot2','./data/robot3']
##################################################ADDDED PART
            self.motors = []
            self.motors.append(Motor('Motor 0', is_debug=False))
            self.motors.append(Motor('Motor 1', is_debug=False))
            self.motors.append(Motor('Motor 2', is_debug=False))
            self.motors.append(Motor('Motor 3', is_debug=False))
            self.motors.append(Motor('Motor 4', is_debug=False))

            self.goal_keeper = 4
            self.lower_attacker = 0
            self.upper_attacker = 1
            self.lower_defender = 2
            self.upper_defender = 3
            self.start = 0

            self.prev_our_postures = np.array([])
            self.prev_action0 = 0

            #self.our_posture = np.array(self.strategy.data_proc.get_my_team_postures()).reshape(-1)
######################################################################################
            return
##############################################################################
            
        try:
            info = yield self.call(u'aiwc.get_info', args.key)
        except Exception as e:
            printWrapper("Error: {}".format(e))
        else:
            printWrapper("Got the game info successfully")
            try:
                self.sub = yield self.subscribe(self.on_event, args.key)
                printWrapper("Subscribed with subscription ID {}".format(self.sub.id))
            except Exception as e2:
                printWrapper("Error: {}".format(e2))
               
        init_variables(self, info)
        
        try:
            yield self.call(u'aiwc.ready', args.key)
        except Exception as e:
            printWrapper("Error: {}".format(e))
        else:
            printWrapper("I am ready for the game!")
            
            
    @inlineCallbacks
    def on_event(self, f):        
        #printWrapper("event received")

        @inlineCallbacks
        def set_wheel(self, robot_wheels):
            yield self.call(u'aiwc.set_speed', args.key, robot_wheels)
            return
        
        # initiate empty frame
        #printWrapper("Initiate emtpy frame")
        received_frame = Frame()
        received_subimages = []
        
        if 'time' in f:
            received_frame.time = f['time']
        if 'score' in f:
            received_frame.score = f['score']
        if 'reset_reason' in f:
            received_frame.reset_reason = f['reset_reason']
        if 'subimages' in f:
            received_frame.subimages = f['subimages']
            # Comment the next lines if you don't need to use the image information
            for s in received_frame.subimages:
                received_subimages.append(SubImage(s['x'],
                                                   s['y'],
                                                   s['w'],
                                                   s['h'],
                                                   s['base64'].encode('utf8')))   
            self.image.update_image(received_subimages)
        if 'coordinates' in f:
            received_frame.coordinates = f['coordinates']            
        if 'EOF' in f:
            self.end_of_frame = f['EOF']     

        if (self.end_of_frame):
            #print("end of frame")
            self._frame += 1 

##############################################################################
            ######################################
            # BEGIN ALGORITHM                    #
            ######################################
            self.data_proc.update_cur_frame(received_frame)
            # <DQL1> Get state, and preprocess
            our_postures = np.array(self.strategy.data_proc.get_my_team_postures()).reshape(-1)
            opponent_postures = np.array(self.strategy.data_proc.get_opponent_postures()).reshape(-1)
            cur_ball = np.array(self.strategy.data_proc.get_cur_ball_position()).reshape(-1)
            if len(self.prev_our_postures) > 0:
                velocity = our_postures - self.prev_our_postures
            else:
                velocity = np.array([0]*15).reshape(-1)
            self.prev_our_postures = our_postures
            x1 = np.concatenate((our_postures, opponent_postures, cur_ball, velocity))
            x1 = np.reshape(x1, [1, self.state_size])

            # <DQL2> Get action number
            
            a0 = self.act(0, x1)
            act_values0 = [0]*12
            act_values0[a0] = 1

            a1 = self.act(1, x1)
            act_values1 = [0]*14
            act_values1[a1] = 1

            a2 = self.act(2, x1)
            act_values2 = [0]*13
            act_values2[a2] = 1

            a3 = self.act(3, x1)
            act_values3 = [0]*17
            act_values3[a3] = 1


            # <DQL3> Perform action and get next state, reward, done. 

            # do some processing with data_proc
            wheels, actions = self.strategy.perform()
            wheels[0], wheels[1] = ActionNumInterpreter(0, self.strategy, act_values0)
            wheels[2], wheels[3] = ActionNumInterpreter(1, self.strategy, act_values1)
            wheels[4], wheels[5] = ActionNumInterpreter(2, self.strategy, act_values2)
            wheels[6], wheels[7] = ActionNumInterpreter(3, self.strategy, act_values3)

            set_wheel(self, wheels)

            # <DQL4> Remember (prev_state, prev_action, cur_reward): cur_reward == +1 only if done==GOAL, cur_reward == -1 only if done==LOSE, 
            if received_frame.reset_reason == SCORE_MYTEAM:
                reward = 100
                self.done = True
                self.my_team_score += 1
                printWrapper("Cumulative Reward is: " + str(self.cumulative_reward))
                with open("./data/cumulative_reward.txt",'a') as ff:
                    ff.write(str(self.cumulative_reward) + "\n")
                self.cumulative_reward = 0
            elif received_frame.reset_reason == SCORE_OPPONENT:
                reward = -100
                self.done = True
                self.opp_team_score += 1
                printWrapper("Cumulative Reward is: " + str(self.cumulative_reward))
                with open("./data/cumulative_reward.txt",'a') as ff:
                    ff.write(str(self.cumulative_reward) + "\n")
                self.cumulative_reward = 0
            elif received_frame.reset_reason == DEADLOCK:
                reward = 0
                self.done = True
                printWrapper("Cumulative Reward is: " + str(self.cumulative_reward))
                with open("./data/cumulative_reward.txt",'a') as ff:
                    ff.write(str(self.cumulative_reward) + "\n")
                self.cumulative_reward = 0
            else: # No reset, just go on
                self.done = False
                reward = -1
                self.cumulative_reward += reward
            
            if not self.cnt == 0:
                self.remember(0,[self.prev_state, self.prev_action0, reward])
                self.remember(1,[self.prev_state, self.prev_action0, reward])
                self.remember(2,[self.prev_state, self.prev_action0, reward])
                self.remember(3,[self.prev_state, self.prev_action0, reward])

            self.prev_state = x1
            self.prev_action0 = a0
            self.prev_action1 = a1
            self.prev_action2 = a2
            self.prev_action3 = a3

            # <DQL5> If done == True: replay(), or train the Q network with Bellman optimality equation
            if self.done == True or self.cnt > 300:
                self.done = False
                self.cnt  = 0
                if len(self.memory3) >= self.batch_size:
                    self.replay(0, self.batch_size)
                    self.replay(1, self.batch_size)
                    self.replay(2, self.batch_size)
                    self.replay(3, self.batch_size)
                    
                printWrapper("New Episode! New Epsilon: " + str(self.epsilon))
            else:
                self.cnt += 1

            self.losscounter += 1

            # Save weights
            if self.losscounter % 10000 == 0 and self.losscounter > 1:
                self.save(0, "./save/dqn0.h5")
            elif self.losscounter % 10005 == 0 and self.losscounter > 1:
                self.save(1, "./save/dqn1.h5")
            elif self.losscounter % 10010 == 0 and self.losscounter > 1:
                self.save(2, "./save/dqn2.h5")
            elif self.losscounter % 10015 == 0 and self.losscounter > 1:
                self.save(3, "./save/dqn3.h5")

            # Save q values
            if self.losscounter % 10002 == 0 and self.losscounter > 1:
                self.write_loss(0, self.batch_size)
            elif self.losscounter % 10007 == 0 and self.losscounter > 1:
                self.write_loss(1, self.batch_size)
            elif self.losscounter % 10012 == 0 and self.losscounter > 1:
                self.write_loss(2, self.batch_size)
            elif self.losscounter % 10017 == 0 and self.losscounter > 1:
                self.write_loss(3, self.batch_size)

            # Save score values
            if self.losscounter % 10 == 0:
                with open('./data/scores.txt','a') as ff:
                    ff.write(str(self.my_team_score) + ", " + str(self.opp_team_score) + "\n")
            ######################################
            # END ALGORITHM                      #
            ######################################
##############################################################################            
          
            if(received_frame.reset_reason == GAME_END):
                printWrapper("Game ended.")

##############################################################################
                #(virtual finish() in random_walk.cpp)
                #save your data
                with open(args.datapath + '/result.txt', 'w') as output:
                    #output.write('yourvariables')
                    output.close()
                #unsubscribe; reset or leave  
                yield self.sub.unsubscribe()
                printWrapper("Unsubscribed...")
                self.leave()
##############################################################################
            
            self.end_of_frame = False

    def onDisconnect(self):
        printWrapper("disconnected")
        if reactor.running:
            reactor.stop()


if __name__ == '__main__':
    printWrapper("[MINUK] Start of Main")
    parser = argparse.ArgumentParser()
    parser.add_argument("server_ip")
    parser.add_argument("port")
    parser.add_argument("realm")
    parser.add_argument("key")
    parser.add_argument("datapath")
    
    args = parser.parse_args()
    #print ("Arguments:")
    #print (args.server_ip)
    #print (args.port)
    #print (args.realm)
    #print (args.key)
    #print (args.datapath)
    
    ai_sv = "rs://" + args.server_ip + ":" + args.port
    ai_realm = args.realm
    
    printWrapper("[MINUK] Before making component")
    # create a Wamp session object
    session = Component(ComponentConfig(ai_realm, {}))

    # initialize the msgpack serializer
    serializer = MsgPackSerializer()
    
    # use Wamp-over-rawsocket
    runner = ApplicationRunner(ai_sv, ai_realm, serializers=[serializer])
    
    runner.run(session, auto_reconnect=True)


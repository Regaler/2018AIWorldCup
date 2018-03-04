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
from keras.models import model_from_json
from keras.models import Sequential
from keras.layers.core import Dense, Dropout, Activation, Flatten
from keras.layers.convolutional import Convolution2D, MaxPooling2D
from keras.optimizers import SGD , Adam
import tensorflow as tf
from keras import losses
import keras

from ReplayBuffer import ReplayBuffer
from ActorNetwork import ActorNetwork
from CriticNetwork import CriticNetwork
from OU import OU

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

OU = OU()       #Ornstein-Uhlenbeck Process

def printWrapper(msg):
    print(msg)
    sys.__stdout__.flush() # stdout is redirected to somewhere else. flush __stdout__ directly.
        
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

        self.epsilon = 0.5
        self.final_epsilon = 0.01 # Final epsilon value
        self.dec_epsilon = 0.001 # Decrease rate of epsilon for every generation
        self.gamma = 0.99 # 0.99
        self.batch_size = 32
        self.learning_rate = 1e-5

        self.cnt = 0
        self.state_size = 40
        self.label_size = 10

        #Tensorflow GPU optimization
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        sess = tf.Session(config=config)
        from keras import backend as K
        K.set_session(sess)
        self.actor = ActorNetwork(sess, self.state_size, self.label_size, self.batch_size, TAU=0.001, LRA=0.0001)
        self.critic = CriticNetwork(sess, state_dim, action_dim, self.batch_size, TAU=0.001, LRC=0.001)
        self.buff = ReplayBuffer(BUFFER_SIZE=10000)    #Create replay buffer

        #Now load the weight
        print("Now we load the weight")
        try:
            actor.model.load_weights("./save/actormodel.h5")
            critic.model.load_weights("./save/criticmodel.h5")
            actor.target_model.load_weights("./save/actormodel.h5")
            critic.target_model.load_weights("./save/criticmodel.h5")
            print("Weight load successfully")
        except:
            print("Cannot find the weight")

        self.done = False
        self.losscounter = 0
        self.average_loss = []
        self.cumulative_reward = 0
        self.my_team_score = 0
        self.opp_team_score = 0

    def onConnect(self):
        printWrapper("Transport connected")
        self.join(self.config.realm)

    """Some methods"""
    def remember(self, state_action_reward_triplet):
        self.buff.append((state_action_reward_triplet[0], state_action_reward_triplet[1], state_action_reward_triplet[2]))

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
            printWrapper("Initializing variables...")
            self.resolution = info['resolution']
            self.image = Received_Image(self.resolution, self.colorChannels)
            self.data_proc = Data_processor(is_debug=False)
            self.strategy = Strategy(self.data_proc, is_debug=False)
##################################################ADDDED PART
            self.motors = []
            self.motors.append(Motor('Motor 0', is_debug=False))
            self.motors.append(Motor('Motor 1', is_debug=False))
            self.motors.append(Motor('Motor 2', is_debug=False))
            self.motors.append(Motor('Motor 3', is_debug=False))
            self.motors.append(Motor('Motor 4', is_debug=False))

            self.prev_our_postures = np.array([])
            self.prev_action = 0

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
            our_postures = np.array(self.data_proc.get_my_team_postures()).reshape(-1)
            opponent_postures = np.array(self.data_proc.get_opponent_postures()).reshape(-1)
            cur_ball = np.array(self.data_proc.get_cur_ball_position()).reshape(-1)
            if len(self.prev_ball) > 0:
                velocity = cur_ball - self.prev_ball
            else:
                velocity = np.array([0]*3).reshape(-1)
            coordinates = np.concatenate((our_postures, opponent_postures, cur_ball, velocity))

            # <DQL2> Select action with actor. Add some randomness with OU process.
            a_t = np.zeros([1, self.label_size])
            noise_t = np.zeros([1, self.label_size])

            a_t_original = actor.model.predict(s_t.reshape(1, s_t.shape[0]))
            for i in range(10):
                noise_t[0][i] = max(epsilon, 0) * OU.function(a_t_original[0][i], 0.5, 0.60, 0.30)
                a_t[0][i] = a_t_original[0][i] + noise_t[0][i]

            # <DQL3> Perform action and get next state, reward, done. 
            wheels = a_t
            set_wheel(self, wheels)

            # <DQL4> Remember (prev_state, prev_action, cur_reward): cur_reward == +1 only if done==GOAL, cur_reward == -1 only if done==LOSE, 
            if received_frame.reset_reason == SCORE_MYTEAM:
                reward = 100
                self.done = True
                self.my_team_score += 1
                self.cumulative_reward = 0
            elif received_frame.reset_reason == SCORE_OPPONENT:
                reward = -100
                self.done = True
                self.opp_team_score += 1
                self.cumulative_reward = 0
            elif received_frame.reset_reason == DEADLOCK:
                reward = 0
                self.done = True
                self.cumulative_reward = 0
            else: # No reset, just go on
                self.done = False
                if cur_ball[0] < -1.5:
                    reward = -3
                elif cur_ball[0] < -1.0:
                    reward = -2.5
                elif cur_ball[0] < -0.5:
                    reward = -2.0
                elif cur_ball[0] < 0:
                    reward = -1.5
                elif cur_ball[0] < 0.5:
                    reward = -1.0
                elif cur_ball[0] < 1.0:
                    reward = -0.5
                elif cur_ball[0] < 1.5:
                    reward = -0.2
                else:
                    reward = 0
                self.cumulative_reward += reward
            
            if not self.cnt == 0:
                self.remember(0,[self.prev_state, self.prev_action, reward, coordinates, self.done])

            self.prev_state = coordinates

            # <DQL5> If done == True: replay(), or train the Q network with Bellman optimality equation
            #Do the batch update
            batch = buff.getBatch(BATCH_SIZE)
            states = np.asarray([e[0] for e in batch])
            actions = np.asarray([e[1] for e in batch])
            rewards = np.asarray([e[2] for e in batch])
            new_states = np.asarray([e[3] for e in batch])
            dones = np.asarray([e[4] for e in batch])
            y_t = np.asarray([e[1] for e in batch])

            target_q_values = critic.target_model.predict([new_states, actor.target_model.predict(new_states)])

            for k in range(len(batch)):
                if dones[k]:
                    y_t[k] = rewards[k]
                else:
                    y_t[k] = rewards[k] + GAMMA*target_q_values[k]

            loss += critic.model.train_on_batch([states,actions], y_t) 
            a_for_grad = actor.model.predict(states)
            grads = critic.gradients(states, a_for_grad)
            actor.train(states, grads)
            actor.target_train()
            critic.target_train()

            print("Now we save model")
            if self.losscounter % 10000 == 0 and self.losscounter > 1:
                actor.model.save_weights("./save/actormodel.h5", overwrite=True)
                critic.model.save_weights("./save/criticmodel.h5", overwrite=True)
                with open('./data/scores.txt','a') as ff:
                    ff.write(str(self.my_team_score) + ", " + str(self.opp_team_score) + "\n")

            self.losscounter += 1
            self.cumulative_reward += reward
            self.epsilon = max(self.epsilon - self.dec_epsilon, self.final_epsilon)
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


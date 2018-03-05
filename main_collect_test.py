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

from data_processor import *
from strategy_v17 import *
import math
from PIL import Image
import numpy as np

import os
import base64

#reset_reason
NONE = 0
GAME_START = 1
SCORE_MYTEAM = 2
SCORE_OPPONENT = 3
GAME_END = 4

#coordinates
MY_TEAM = 0
OP_TEAM = 1
BALL = 2
X = 0
Y = 1
TH = 2

COORDINATION0 = './data/test/COORDINATION0.pkl'
COORDINATION1 = './data/test/COORDINATION1.pkl'
COORDINATION2 = './data/test/COORDINATION2.pkl'
COORDINATION3 = './data/test/COORDINATION3.pkl'
LABEL0 = './data/test/label0.pkl'
LABEL1 = './data/test/label1.pkl'
LABEL2 = './data/test/label2.pkl'
LABEL3 = './data/test/label3.pkl'
#IMAGES = './data/images/'

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
        printWrapper("1111")
        self.resolution = resolution
        self.colorChannels = colorChannels
        # need to initialize the matrix at timestep 0
        self.ImageBuffer = np.zeros((resolution[1], resolution[0], colorChannels)) # rows, columns, colorchannels
        printWrapper("2222")
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
        

    def onConnect(self):
        printWrapper("Transport connected")
        self.join(self.config.realm)

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
            self.colorChannels = 3 # nf
            self.end_of_frame = False
            self.image = Received_Image(self.resolution, self.colorChannels)
            printWrapper("Initializing variables...")
            self.data_proc = Data_processor(is_debug=False)

            self.strategy = Strategy(self.data_proc, is_debug=False)
            self._frame = 0 
            self.wheels = [0 for _ in range(10)]   
            self.cnt = 0
            self.state_cnt = 0
            self.path = ['./data/robot0','./data/robot1','./data/robot2','./data/robot3']
            self.prev_our_postures = []
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
        printWrapper("event received")

        @inlineCallbacks
        def set_wheel(self, robot_wheels):
            yield self.call(u'aiwc.set_speed', args.key, robot_wheels)
            return
        
        # initiate empty frame
        printWrapper("Initiate emtpy frame")
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
            #(virtual update() in random_walk.cpp)
            #wheels = [random.uniform(-self.max_linear_velocity,self.max_linear_velocity) for _ in range(10)]
            ######################################
            # BEGIN ALGORITHM                    #
            ######################################
            # <1> Get and calculate state
            self.data_proc.update_cur_frame(received_frame)
            our_postures = np.array(self.data_proc.get_my_team_postures()).reshape(-1)
            opponent_postures = np.array(self.data_proc.get_opponent_postures()).reshape(-1)
            cur_ball = np.array(self.data_proc.get_cur_ball_position()).reshape(-1)
            transition = np.array(self.data_proc.get_cur_ball_transition()).reshape(-1)
           
            coordination = np.concatenate((our_postures, opponent_postures, cur_ball, transition))

            # <2> Perform and get actions
            wheels, cur_actions = self.strategy.perform() # perform
            actions_one_hot_encoded = hot_encode(cur_actions) # one hot encoding

            # <3> Write some values for supervised learning
            if (not os.path.exists(LABEL0)) and (self.cnt % 4 == 0):
                with open(COORDINATION0,'wb') as hh:
                    temp_coordination = []
                    temp_coordination.append(coordination)
                    pickle.dump(temp_coordination,hh)
                with open(LABEL0,'wb') as ff:
                    templist0 = []
                    templist0.append(actions_one_hot_encoded[0])
                    #printWrapper(templist)
                    pickle.dump(templist0, ff)
            elif (not os.path.exists(LABEL1)) and (self.cnt % 4 == 1):
                with open(COORDINATION1,'wb') as hh:
                    temp_coordination = []
                    temp_coordination.append(coordination)
                    pickle.dump(temp_coordination,hh)
                with open(LABEL1,'wb') as ff:
                    templist1 = []
                    templist1.append(actions_one_hot_encoded[1])
                    #printWrapper(templist)
                    pickle.dump(templist1, ff)
            elif (not os.path.exists(LABEL2)) and (self.cnt % 4 == 2):
                with open(COORDINATION2,'wb') as hh:
                    temp_coordination = []
                    temp_coordination.append(coordination)
                    pickle.dump(temp_coordination,hh)
                with open(LABEL2,'wb') as ff:
                    templist2 = []
                    templist2.append(actions_one_hot_encoded[2])
                    #printWrapper(templist)
                    pickle.dump(templist2, ff)
            elif (not os.path.exists(LABEL3)) and (self.cnt % 4 == 3):
                with open(COORDINATION3,'wb') as hh:
                    temp_coordination = []
                    temp_coordination.append(coordination)
                    pickle.dump(temp_coordination,hh)
                with open(LABEL3,'wb') as ff:
                    templist3 = []
                    templist3.append(actions_one_hot_encoded[3])
                    #printWrapper(templist)
                    pickle.dump(templist3, ff)
                
            elif self.cnt % 4 == 0:
                with open(COORDINATION0,'rb') as hh:
                    temp_coordination0=pickle.load(hh)
                    temp_coordination0.append(coordination)
                with open(LABEL0,'rb') as ff:
                    templist0 = pickle.load(ff)
                    templist0.append(actions_one_hot_encoded[0])

                with open(COORDINATION0,'wb') as hh:
                    pickle.dump(temp_coordination0, hh)
                with open(LABEL0,'wb') as ff:
                    pickle.dump(templist0, ff)

            elif self.cnt % 4 == 1:
                with open(COORDINATION1,'rb') as hh:
                    temp_coordination1=pickle.load(hh)
                    temp_coordination1.append(coordination)
                with open(LABEL1,'rb') as ff:
                    templist1 = pickle.load(ff)
                    templist1.append(actions_one_hot_encoded[1])

                with open(COORDINATION1,'wb') as hh:
                    pickle.dump(temp_coordination1, hh)
                with open(LABEL1,'wb') as ff:
                    pickle.dump(templist1, ff)

            elif self.cnt % 4 == 2:
                with open(COORDINATION2,'rb') as hh:
                    temp_coordination2=pickle.load(hh)
                    temp_coordination2.append(coordination)
                with open(LABEL2,'rb') as ff:
                    templist2 = pickle.load(ff)
                    templist2.append(actions_one_hot_encoded[2])

                with open(COORDINATION2,'wb') as hh:
                    pickle.dump(temp_coordination2, hh)
                with open(LABEL2,'wb') as ff:
                    pickle.dump(templist2, ff)

            elif self.cnt % 4 == 3:
                with open(COORDINATION3,'rb') as hh:
                    temp_coordination3=pickle.load(hh)
                    temp_coordination3.append(coordination)
                with open(LABEL3,'rb') as ff:
                    templist3 = pickle.load(ff)
                    templist3.append(actions_one_hot_encoded[3])

                with open(COORDINATION3,'wb') as hh:
                    pickle.dump(temp_coordination3, hh)
                with open(LABEL3,'wb') as ff:
                    pickle.dump(templist3, ff)
                
            else:
                pass

            # <4>
            self.cnt+=1
            set_wheel(self, wheels)
            self.prev_our_postures = our_postures
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
    
    # create a Wamp session object
    session = Component(ComponentConfig(ai_realm, {}))

    # initialize the msgpack serializer
    serializer = MsgPackSerializer()
    
    # use Wamp-over-rawsocket
    runner = ApplicationRunner(ai_sv, ai_realm, serializers=[serializer])
    
    runner.run(session, auto_reconnect=True)
#!/usr/bin/python3
from twisted.internet import reactor
from twisted.internet.defer import inlineCallbacks

from autobahn.wamp.serializer import MsgPackSerializer
from autobahn.wamp.types import ComponentConfig
from autobahn.twisted.wamp import ApplicationSession, ApplicationRunner

import argparse
import random

from data_processor import *
from strategy import *
import math
import sys

#******************************************************************************#
from strategy_v13 import *
from strategy_v17_just_parameter_value_revised import *
#******************************************************************************#

def printWrapper(msg):
    print(msg)
    sys.__stdout__.flush() # stdout is redirected to somewhere else. flush __stdout__ directly.


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
        print("Transport connected")
        self.join(self.config.realm)

    @inlineCallbacks
    def onJoin(self, details):
        print("session attached")

##############################################################################
        def init_variables(self, info):
            # Here you have the information of the game (virtual init() in random_walk.cpp)
            # List: game_time, goal, number_of_robots, penalty_area, codewords,
            #       robot_size, max_linear_velocity, field, team_info,
            #       {rating, name}, axle_length, resolution, ball_radius
            # self.game_time = info['game_time']
            # self.field = info['field']
            self.max_linear_velocity = info['max_linear_velocity']
            self.end_of_frame = False
            #print("Initializing variables...")
            printWrapper("Initializing variables...")
            self.data_proc = Data_processor(is_debug=False)
            self.strategy = Strategy(self.data_proc, is_debug=False)


            #*******************************************************************#
            self.strategy_v13 = Strategy_v13(self.data_proc, is_debug=False)
#            self.strategy_v17_just_parameter_value_revised = Strategy_v17(self.data_proc, is_debug=False)

            self.strategy_num = 0
            self.strategy_point = [0] * 2   # 2: number of strategy which we will use
            self.goal_time = 0
            #*******************************************************************#
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
        # print("event received")

        @inlineCallbacks
        def set_wheel(self, robot_wheels):
            yield self.call(u'aiwc.set_speed', args.key, robot_wheels)
            return
        
        # initiate empty frame
        received_frame = Frame()
        
        if 'time' in f:
            received_frame.time = f['time']
        if 'score' in f:
            received_frame.score = f['score']
        if 'reset_reason' in f:
            received_frame.reset_reason = f['reset_reason']
        if 'subimages' in f:
            received_frame.subimages = f['subimages']
        if 'coordinates' in f:
            received_frame.coordinates = f['coordinates']            
        if 'EOF' in f:
            self.end_of_frame = f['EOF']
            
        #print(received_frame.time)
        #print(received_frame.score)
        #print(received_frame.reset_reason)
        #print(self.end_of_frame)
        
        # How to get the robot and ball coordinates: (ROBOT_ID can be 0,1,2,3,4)
        # Available after the 5th event received...
        #print(received_frame.coordinates[MY_TEAM][ROBOT_ID][X])
        #print(received_frame.coordinates[MY_TEAM][ROBOT_ID][Y])
        #print(received_frame.coordinates[MY_TEAM][ROBOT_ID][TH])
        #print(received_frame.coordinates[OP_TEAM][ROBOT_ID][X])
        #print(received_frame.coordinates[OP_TEAM][ROBOT_ID][Y])
        #print(received_frame.coordinates[OP_TEAM][ROBOT_ID][TH])
        #print(received_frame.coordinates[BALL][X])
        #print(received_frame.coordinates[BALL][Y])
        
        if (self.end_of_frame):
            #print("end of frame")

##############################################################################
            #(virtual update() in random_walk.cpp)
            #wheels = [random.uniform(-self.max_linear_velocity,self.max_linear_velocity) for _ in range(10)]
            ######################################
            # BEGIN ALGORITHM                    #
            ######################################
            self.data_proc.update_cur_frame(received_frame)


            #<1> select strategy   
            if received_frame.time < 50:
                if (received_frame.reset_reason == 2) or (received_frame.reset_reason == 3): # goal
                    self.strategy_num = random.randrange(0,2)
                else :
                    pass
                printWrapper("1. strategy_num : " + str(self.strategy_num))
            else:
                self.strategy_num = self.strategy_point.index(max(self.strategy_point))
                printWrapper("2. strategy_num : " + str(self.strategy_num))
            
            #<2> strategy perform
            if self.strategy_num == 0:
                wheels = self.strategy_v13.perform()
            elif self.strategy_num == 1:
                wheels = self.strategy.perform()
                #wheels = self.strategy_v17.perform()
            else:
                printWrapper("ERROR : There is no proper strategy")


            #<3> set wheel
            set_wheel(self, wheels)

            ######################################
            # END ALGORITHM                      #
            ######################################
##############################################################################            

            #<4> calculate score point
            if received_frame.reset_reason == 2: #score my team
                self.strategy_point[self.strategy_num] += 1/(received_frame.time - self.goal_time)
                self.goal_time = received_frame.time
            elif received_frame.reset_reason == 3: #score opponent
                self.strategy_point[self.strategy_num] -= 1/(received_frame.time - self.goal_time)
                self.goal_time = received_frame.time
            else:
                pass
            printWrapper("strategy_point : " + str(self.strategy_point))




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
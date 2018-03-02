from __future__ import division
from __future__ import print_function
import math
from motor import *
from calculator import *
from data_processor import *
import sys
import random

def printWrapper(msg):
    print(msg)
    sys.__stdout__.flush() # stdout is redirected to somewhere else. flush __stdout__ directly.


class Strategy(object):
    def __init__(self, data_proc, is_debug=False, memory = []):
        self.is_debug = is_debug
        if self.is_debug:
            print('Strategy contructed')
        self.cal = Calculator(is_debug=False)
        self.data_proc = data_proc

        self.motors = []
        self.motors.append(Motor('Motor 0', is_debug=False))
        self.motors.append(Motor('Motor 1', is_debug=False))
        self.motors.append(Motor('Motor 2', is_debug=False))
        self.motors.append(Motor('Motor 3', is_debug=False))
        self.motors.append(Motor('Motor 4', is_debug=False))

        self.goal_keeper = 4
        self.upper_attacker = 1
        self.lower_attacker = 0
        self.upper_defender = 3
        self.lower_defender = 2

    def perform(self):
        robot_wheels = [0]*NUMBER_OF_ROBOTS*2
        robot_wheels[self.upper_attacker*2:self.upper_attacker*2+2] = self.attack(self.upper_attacker)
        robot_wheels[self.lower_attacker*2:self.lower_attacker*2+2] = self.attack(self.lower_attacker)
        robot_wheels[self.upper_defender*2:self.upper_defender*2+2] = self.defense(self.upper_defender)
        robot_wheels[self.lower_defender*2:self.lower_defender*2+2] = self.defense(self.lower_defender)
        robot_wheels[self.goal_keeper*2:self.goal_keeper*2+2] = self.keep_goal(self.goal_keeper)
        return robot_wheels

    def attack(self, id):
        # Initializations
        our_postures = self.data_proc.get_my_team_postures()
        my_posture = our_postures[id]
        cur_ball = self.data_proc.get_cur_ball_position()
        cur_trans = self.data_proc.get_cur_ball_transition()
        theta_to_ball = self.cal.compute_theta_to_target(my_posture, cur_ball)
        static_theta = self.cal.compute_static_theta(my_posture, cur_ball)
        dist_to_ball = self.cal.get_distance(my_posture,cur_ball)
        x = my_posture[0]
        y = my_posture[1]
        th = my_posture[2]
        bx = cur_ball[0]
        by = cur_ball[1]
        # ATTACK state
        if bx > -ATTACK_MARGIN: # little margin
            up_pad = FIELD_WIDTH/2 - 0.3
            down_pad = -FIELD_WIDTH/2 + 0.3
            right_pad = FIELD_LENGTH/2 - 0.3
            left_pad = -FIELD_LENGTH/2 + 0.3
            # Corner
            if by > up_pad or by < down_pad or bx > right_pad or bx < left_pad:
                #----------------------------------------------------------------#
                NUM_OF_PREDICTED_FRAMES = 2
                target = [cur_ball[0] + cur_trans[0]*NUM_OF_PREDICTED_FRAMES,
                          cur_ball[1] + cur_trans[1]*NUM_OF_PREDICTED_FRAMES]
                static_theta2 = self.cal.compute_static_theta(my_posture, target)

                # ball is inner area now
                if ((static_theta >= math.pi/2 or static_theta <= -math.pi/2) and (y >= by-0.15 and y <= by+0.15) and x > bx): 
                    # ball is inner area NOW, inner area in FUTURE
                    if ((static_theta2 >= math.pi/2 or static_theta2 <= -math.pi/2) and (y >= by-0.15 and y <= by+0.15) and x > bx):
                        ###printWrapper("INNER-INNER CORNER")
                        if y<by:
                            wheel_velos = self.motors[id].move_to_target(my_posture, [x,y-1])
                        else:
                            wheel_velos = self.motors[id].move_to_target(my_posture, [x,y+1])
                    # ball is inner area NOW, but outer area in FUTURE
                    else:
                        ###printWrapper("INNER-OUTER CORNER")
                        # ball is close : escape to back
                        if dist_to_ball <= 0.4:
                            if y+0.6 >= 1.4: # wall is at North
                                    wheel_velos = self.motors[id].move_to_target(my_posture, [x,y-1])
                            elif y-0.6 <= -1.4: # wall is at South
                                wheel_velos = self.motors[id].move_to_target(my_posture, [x,y+1])
                            else: # In the middle
                                if round(random.random()) == 0:
                                    wheel_velos = self.motors[id].move_to_target(my_posture, [x,y-1])
                                else:
                                    wheel_velos = self.motors[id].move_to_target(my_posture, [x,y+1])
                        # ball is far : escape to specific point
                        else:
                            if y >= 0:
                                wheel_velos = self.motors.three_phase_move_to_target(my_posture, [(x+bx)/2, (y-1.5)/2])
                            else:
                                wheel_velos = self.motors.three_phase_move_to_target(my_posture, [(x+bx)/2, (y+1.5)/2])
                # ball is in outer area, so PUSH to goalpstn
                else: 
                #----------------------------------------------------------------#
                    # ball is at opponent's corner : escape using back rotation
                    if bx > right_pad: 
                        ###printWrapper("RIGHT CORNER")
                        if (y>=0) and (x <= 2.1 and x >= 1.6) and (self.cal.r2d(th) >= 20 and self.cal.r2d(th) <= 160): # Blocking top right corner
                            wheel_velos = [-1.5, -1.8]
                        elif (y>=0) and (y <= 1.5 and y >= 1.2) and (self.cal.r2d(th) >= 180-40 or self.cal.r2d(th) <= -180+40): # Blocking top corner
                            wheel_velos = [-1.5, -1.8]
                        elif (y<0) and (x <= 2.1 and x >= 1.6) and (self.cal.r2d(th) >= -90-20 and self.cal.r2d(th) <= -90+20): # Blocking bottom right corner
                            wheel_velos = [-1.8, -1.5]
                        elif (y<0) and (y >= -1.5 and y <= -1.2) and (self.cal.r2d(th) >= 180-40 or self.cal.r2d(th) <= -180+40): # Blocking bottom corner
                            wheel_velos = [-1.8, -1.5]
                        else:
                            wheel_velos = self.motors[id].move_to_target(my_posture, cur_ball)
                    # ball is at other corner
                    else:
                        ###printWrapper("OTHER CORNER")
                        # ball is close : escape using back rotation
                        if dist_to_ball <= 0.4:
                            if ((static_theta >= math.pi/2 or static_theta <= -math.pi/2) and (y >= by-0.15 and y <= by+0.15) and x > bx):
                                if (y <= 1.5 and y >= 1.2) and (self.cal.r2d(th) >= 180-70 or self.cal.r2d(th) <= -180+70): # Blocking top corner
                                    wheel_velos = [-1.5, -1.8]
                                elif (y >= -1.5 and y <= -1.2) and (self.cal.r2d(th) >= 180-70 or self.cal.r2d(th) <= -180+70 ): # Blocking bottom corner
                                    wheel_velos = [-1.8, -1.5]
                                else:
                                    wheel_velos = self.motors[id].move_to_target(my_posture, cur_ball)
                            else:
                                wheel_velos = self.motors[id].move_to_target(my_posture, cur_ball)
                        # ball is far : escape to specific point
                        else: 
                            if ((static_theta >= math.pi/2 or static_theta <= -math.pi/2) and (y >= by-0.15 and y <= by+0.15) and x > bx): # Ball is in inner area
                                if y >= 0:
                                    wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, [-1.8, -1.4])
                                else:
                                    wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, [-1.8, +1.4])
                            else:
                                wheel_velos = self.motors[id].move_to_target(my_posture, [bx,by])
                    
            # Inside ground
            else: 
                ###printWrapper("INSIDE GROUND")
                NUM_OF_PREDICTED_FRAMES = 2
                target = [cur_ball[0] + cur_trans[0]*NUM_OF_PREDICTED_FRAMES,
                          cur_ball[1] + cur_trans[1]*NUM_OF_PREDICTED_FRAMES]
                static_theta2 = self.cal.compute_static_theta(my_posture, target)

                # ball is inner area now
                if ((static_theta >= math.pi/2 or static_theta <= -math.pi/2) and (y >= by-0.15 and y <= by+0.15) and x > bx): 
                    # ball is inner area NOW, inner area in FUTURE
                    if ((static_theta2 >= math.pi/2 or static_theta2 <= -math.pi/2) and (y >= by-0.15 and y <= by+0.15) and x > bx):
                        ###printWrapper("INNER-INNER GROUND")
                        if y<by:
                            wheel_velos = self.motors[id].move_to_target(my_posture, [x,y-1])
                        else:
                            wheel_velos = self.motors[id].move_to_target(my_posture, [x,y+1])
 
                    # ball is inner area NOW, but outer area in FUTURE
                    else:
                        ###printWrapper("INNER-OUTER GROUND")
                        # ball is close : escape to back
                        if dist_to_ball <= 0.4:
                            if y+0.6 >= 1.4: # wall is at North
                                    wheel_velos = self.motors[id].move_to_target(my_posture, [x,y-1])
                            elif y-0.6 <= -1.4: # wall is at South
                                wheel_velos = self.motors[id].move_to_target(my_posture, [x,y+1])
                            else: # In the middle
                                if round(random.random()) == 0:
                                    wheel_velos = self.motors[id].move_to_target(my_posture, [x,y-1])
                                else:
                                    wheel_velos = self.motors[id].move_to_target(my_posture, [x,y+1])
                        # ball is far : escape to specific point
                        else:
                            if y >= 0:
                                wheel_velos = self.motors.three_phase_move_to_target(my_posture, [(x+bx)/2, (y-1.5)/2])
                            else:
                                wheel_velos = self.motors.three_phase_move_to_target(my_posture, [(x+bx)/2, (y+1.5)/2])
                # ball is in outer area, so PUSH to goalpstn
                else: 
                    ###printWrapper("OUTER-OUTER GROUND")
                    cur_trans = self.data_proc.get_cur_ball_transition()
                    goal_pstn = [1.9, 0]
                    tar_posture = self.cal.compute_desired_posture(cur_ball, cur_trans, goal_pstn)
                    tar_posture = [tar_posture[0]-0.08,tar_posture[1]-0.08,0]
                    wheel_velos = self.motors[id].move_to_target(my_posture, tar_posture, damping=0)

        # DEFEND state
        else: 
            if id == self.upper_attacker:
                if (bx <= -1.5) and (by >= 0):
                    if (x >= -1.9-0.05 and x <= -1.8) and (y >= GOAL_WIDTH+0.2-0.1 and y <= GOAL_WIDTH+0.2+0.1): # Arrived at defense position
                        if th >= self.cal.d2r(90-5) and th <= self.cal.d2r(90+5): # Arrived and angle is good, then PUSH
                            if dist_to_ball <= 0.7 and (bx >= -1.8-0.1 and bx <= -1.8+0.1) and by >= PENALTY_AREA_WIDTH/2+0.1:
                                wheel_velos = [1.8, 1.8]
                            else: # Arrivied but ball is far, then WAIT
                                wheel_velos = [0, 0]
                        else: # Arrived but angle is bad, then TURN
                            wheel_velos = self.motors[id].spin_to_theta(th,self.cal.d2r(90))
                    else:
                        #wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, [-1.9, PENALTY_AREA_WIDTH/2+0.01, math.pi/2])
                        wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, [-1.85, GOAL_WIDTH+0.2, math.pi/2])
                elif (bx <= -0.5): #by<0 : wait
                    wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, [-1, GOAL_WIDTH+0.2, static_theta])
                else:
                    wheel_velos = self.motors[id].move_to_target(my_posture, cur_ball)
                    
            elif id == self.lower_attacker:
                if (bx <= -1.5) and (by < 0):
                    if (x >= -1.9-0.05 and x <= -1.8) and (y >= -GOAL_WIDTH-0.2-0.1 and y <= -GOAL_WIDTH-0.2+0.1): # Arrived at defense position
                        if th >= self.cal.d2r(-90-5) and th <= self.cal.d2r(-90+5): # Arrived and angle is good, then PUSH
                            if dist_to_ball <= 0.7 and (bx >= -1.8-0.1 and bx <= -1.8+0.1) and by <= -PENALTY_AREA_WIDTH/2-0.1:
                                wheel_velos = [1.8, 1.8]
                            else: # Arrivied but ball is far, then WAIT
                                wheel_velos = [0, 0]
                        else: # Arrived but angle is bad, then TURN
                            wheel_velos = self.motors[id].spin_to_theta(th,self.cal.d2r(-90))
                    else: 
                        #swheel_velos = self.motors[id].three_phase_move_to_target(my_posture, [-1.9, -PENALTY_AREA_WIDTH/2-0.01, 3*math.pi/2])
                        wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, [-1.85, -GOAL_WIDTH-0.2, math.pi/2])
                elif (bx <= -0.5):
                    wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, [-1, -GOAL_WIDTH-0.2, static_theta])
                else: # In defense state, if ball is in outer area than the robot, follow the ball.
                    wheel_velos = self.motors[id].move_to_target(my_posture, cur_ball)
        return wheel_velos

    def defense(self, id):
        our_postures = self.data_proc.get_my_team_postures()
        my_posture = our_postures[id]
        cur_ball = self.data_proc.get_cur_ball_position()
        cur_trans = self.data_proc.get_cur_ball_transition()
        theta_to_ball = self.cal.compute_theta_to_target(my_posture, cur_ball)
        static_theta = self.cal.compute_static_theta(my_posture, cur_ball)
        dist_to_ball = self.cal.get_distance(my_posture,cur_ball)

        x = my_posture[0]
        y = my_posture[1]
        th = my_posture[2]
        bx = cur_ball[0]
        by = cur_ball[1]
        wait_x = -1.85
        if (by >= GOAL_WIDTH-0.1):
            wait_y = GOAL_WIDTH-0.1
        elif (by <= -GOAL_WIDTH+0.1):
            wait_y = -GOAL_WIDTH+0.1
        else: 
            wait_y = by

        if bx < -DEFENSE_MARGIN: # DEFENSE state
            ox = 0.05
            cur_trans = self.data_proc.get_cur_ball_transition()
            NUM_OF_PREDICTED_FRAMES = 1
            target = [cur_ball[0] + cur_trans[0]*NUM_OF_PREDICTED_FRAMES,
                      cur_ball[1] + cur_trans[1]*NUM_OF_PREDICTED_FRAMES]
            wheel_velos = self.motors[id].move_to_target(my_posture, target, damping=0.2)

            # ball is coming                                      0.2
            if (target[0] < wait_x+0.05 and target[1]<GOAL_WIDTH+0.4 and target[1]>-GOAL_WIDTH-0.4): 
                # 1. Emergency
                if (target[0] < wait_x+0.03 and target[1]<GOAL_WIDTH+0.05 and target[1]>-GOAL_WIDTH-0.05):
                    wheel_velos = self.motors[id].move_to_target(my_posture, [wait_x-0.02, wait_y], damping=0)
                # 2. Up Coming
                elif (bx<=wait_x+0.07 and by>=GOAL_WIDTH-0.1):
                    if id == self.upper_defender:
                        if (x<=bx) and (y<=by-0.05) and (th>=self.cal.d2r(90) and th<=self.cal.d2r(100)): # position and angle is good: PUSH!!
                            wheel_velos = self.motors[id].move_to_target(my_posture, [min(wait_x,bx),by], damping=0)
                        elif (x<=bx) and (y<=by-0.05): #position is good, but angle is bad: TURN!!
                            wheel_velos = self.motors[id].spin_to_theta(th, self.cal.d2r(90))
                        else: # position is bad
                            if (dist_to_ball<=0.15): # close so rotate and rechase
                                wheel_velos = self.motors[id].move_to_target(my_posture, [target[0]+0.2,target[1]-0.05], damping=0)
                            else:  
                                wheel_velos = self.motors[id].move_to_target(my_posture, [min(wait_x,target[0]-0.05),target[1]-0.05], damping=0)
                    elif id == self.lower_defender:
                        wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, [-1.0, min(0.0, cur_ball[1]), static_theta])                          
                # 3. Down Coming
                elif (bx<=wait_x+0.07 and by<=-GOAL_WIDTH+0.1):
                    if id == self.upper_defender:
                        wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, [-1.0, min(0.0, cur_ball[1]), static_theta])
                    elif id == self.upper_defender:
                        if (x<=bx) and (y>=by+0.05) and (th<=self.cal.d2r(-90) and th>=self.cal.d2r(-100)): # position and angle is good: PUSH!!
                            wheel_velos = self.motors[id].move_to_target(my_posture, [min(wait_x,bx),by], damping=0)
                        elif (x<=bx) and (y>=by+0.05): #position is good, but angle is bad: TURN!!
                            wheel_velos = self.motors[id].spin_to_theta(th, self.cal.d2r(-90))
                        else: # position is bad
                            if (dist_to_ball<=0.15): # close so rotate and rechase
                                wheel_velos = self.motors[id].move_to_target(my_posture, [target[0]+0.2,target[1]+0.05], damping=0)
                            else:
                                wheel_velos = self.motors[id].move_to_target(my_posture, [min(wait_x,target[0]-0.05),target[1]+0.05], damping=0)
                # 4.
                else:
                    cur_trans = self.data_proc.get_cur_ball_transition()
                    wheel_velos = self.motors[id].move_to_target(my_posture, target, damping=0.1)

            # ball is inner area now, and close
            elif ((static_theta >= math.pi/2 or static_theta <= -math.pi/2) and (y >= by-0.07 and y <= by+0.07) and x > bx) and (dist_to_ball <=0.4): 
                if y<by:
                    wheel_velos = self.motors[id].move_to_target(my_posture, [x,y-1])
                else:
                    wheel_velos = self.motors[id].move_to_target(my_posture, [x,y+1])
           
            # ball is outer area now
            else:
                if(dist_to_ball <= 0.15): # ball is close
                    cur_trans = self.data_proc.get_cur_ball_transition()
                    goal_pstn = [1.9, 0]
                    tar_posture = self.cal.compute_desired_posture(cur_ball, cur_trans, goal_pstn)
                    tar_posture = [tar_posture[0]-0.08,tar_posture[1]-0.08,0]
                    wheel_velos = self.motors[id].move_to_target(my_posture, tar_posture, damping=0)
                else: # ball is far
                    NUM_OF_PREDICTED_FRAMES = 2
                    target = [cur_ball[0] + cur_trans[0]*NUM_OF_PREDICTED_FRAMES,
                              cur_ball[1] + cur_trans[1]*NUM_OF_PREDICTED_FRAMES]
                    wheel_velos = self.motors[id].move_to_target(my_posture, target, damping=0)
        

        else: # ATTACK state
            cur_trans = self.data_proc.get_cur_ball_transition()
            NUM_OF_PREDICTED_FRAMES = 1
            target = [cur_ball[0] + cur_trans[0]*NUM_OF_PREDICTED_FRAMES,
                      cur_ball[1] + cur_trans[1]*NUM_OF_PREDICTED_FRAMES]
            #GAEDOL
            if (target[0] <= 2.1 and target[0] >= 1.4 and x >= 1.2) and (target[1] >= -0.4 and target[1] <= 0.4) :
                                                                                                        #if (bx <= 2.1 and bx >= 1.4 and x >= 1.2) and (by >= -0.4 and by <= 0.4) :
                if dist_to_ball <= 0.15:
                    wheel_velos = self.motors[id].move_to_target(my_posture, cur_ball)
                else:
                    # ball is coming up
                    if cur_trans[1] > 0: 
                        if id == self.upper_defender:
                            #wheel_velos = self.motors[id].move_to_target(my_posture, [1.8, 0])
                            NUM_OF_PREDICTED_FRAMES = 3
                            target = [cur_ball[0] + cur_trans[0]*NUM_OF_PREDICTED_FRAMES,
                            cur_ball[1] + cur_trans[1]*NUM_OF_PREDICTED_FRAMES]
                            wheel_velos = self.motors[id].move_to_target(my_posture, target, damping=0.2)
                        elif id == self.lower_defender:
                            #wheel_velos = self.motors[id].move_to_target(my_posture, [1.8, 0])
                            wheel_velos = self.motors[id].move_to_target(my_posture, target, damping=0.2)
                    # ball is coming down
                    else: 
                        if id == self.upper_defender:
                            #wheel_velos = self.motors[id].move_to_target(my_posture, [1.8, 0])
                            wheel_velos = self.motors[id].move_to_target(my_posture, target, damping=0.2)
                        elif id == self.lower_defender:
                            #wheel_velos = self.motors[id].move_to_target(my_posture, [1.8, 0])
                            NUM_OF_PREDICTED_FRAMES = 3
                            target = [cur_ball[0] + cur_trans[0]*NUM_OF_PREDICTED_FRAMES,
                            cur_ball[1] + cur_trans[1]*NUM_OF_PREDICTED_FRAMES]
                            wheel_velos = self.motors[id].move_to_target(my_posture, target, damping=0.2)       
            #GAEDOL x
            else:
                if (y>=0.3) and (x <= 2.1 and x >= 1.7) and (self.cal.r2d(th) >= 60 and self.cal.r2d(th) <= 100): # Blocking top right corner
                    wheel_velos = [-1.5, -1.8]
                elif (y>=0) and (y <= 1.5 and y >= 1.2) and (self.cal.r2d(th) >= 180-40 or self.cal.r2d(th) <= -180+40): # Blocking top corner
                    wheel_velos = [-1.5, -1.8]
                elif (y<-0.3) and (x <= 2.1 and x >= 1.7) and (self.cal.r2d(th) >= -90-40 and self.cal.r2d(th) <= -90+40): # Blocking bottom right corner
                    wheel_velos = [-1.8, -1.5]
                elif (y<0) and (y >= -1.5 and y <= -1.2) and (self.cal.r2d(th) >= 180-40 or self.cal.r2d(th) <= -180+40): # Blocking bottom corner
                    wheel_velos = [-1.8, -1.5]
                else: # NORMAL backup
                    if (x>=1.4-0.05 and x<=1.4+0.05 and y>=-0.15-0.05 and y<=+0.15+0.05) and (th<=self.cal.d2r(20) or th>=self.cal.d2r(-20)) and (dist_to_ball <= 0.15): # Arrived at backup position
                        wheel_velos = [-1.8, -1.8]
                        #elif (dist_to_ball <= 0.15) and (cur_trans[1] < 0) # ball is coming up in inner area
                    else: #Not arrived at backup position
                        if id == self.lower_defender:
                            wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, [1.3, -0.15, 0])
                        elif id == self.upper_defender:
                            wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, [1.3, +0.15, 0])
        return wheel_velos


    def keep_goal(self, id):
        our_postures = self.data_proc.get_my_team_postures()
        my_posture = our_postures[id]
        cur_ball = self.data_proc.get_cur_ball_position()
        theta_to_ball = self.cal.compute_theta_to_target(my_posture, cur_ball)
        static_theta = self.cal.compute_static_theta(my_posture, cur_ball)
        dist_to_ball = self.cal.get_distance(my_posture,cur_ball)

        cur_trans = self.data_proc.get_cur_ball_transition()
        NUM_OF_PREDICTED_FRAMES = 2
        target = [cur_ball[0] + cur_trans[0]*NUM_OF_PREDICTED_FRAMES,
                  cur_ball[1] + cur_trans[1]*NUM_OF_PREDICTED_FRAMES]
        predict_ball_theta = self.cal.compute_static_theta(cur_ball, target)

        x = my_posture[0]
        y = my_posture[1]
        th = my_posture[2]
        bx = cur_ball[0]
        by = cur_ball[1]

        wait_x = -1.85
        if (target[1] >= GOAL_WIDTH-0.1):
            wait_y = GOAL_WIDTH-0.1
        elif (target[1] <= -GOAL_WIDTH+0.1):
            wait_y = -GOAL_WIDTH+0.1
        else: 
            wait_y = target[1]

        # ball is coming from wall
        if (target[0] < wait_x+0.05 and target[1]<GOAL_WIDTH+0.2 and target[1]>-GOAL_WIDTH-0.2): 
            if (target[0] < wait_x+0.03 and target[1]<GOAL_WIDTH+0.05 and target[1]>-GOAL_WIDTH-0.05):
                wheel_velos = self.motors[id].move_to_target(my_posture, [wait_x, wait_y], damping=0)
            # contact with ball at wall
            elif(dist_to_ball <= 0.2): 
                if (x<=bx): # inner side than ball 
                    if (bx<=wait_x+0.07 and by>=GOAL_WIDTH-0.1): # up coming
                        if (y>=GOAL_WIDTH-0.1) and (th>=self.cal.d2r(90) and th<=self.cal.d2r(100)): # position and angle is good: PUSH!!
                            wheel_velos = self.motors[id].move_to_target(my_posture, [min(wait_x,bx),by], damping=0)
                        elif (y>=GOAL_WIDTH-0.1): #position is good, but angle is bad: TURN!!
                            wheel_velos = self.motors[id].spin_to_theta(th, self.cal.d2r(90))
                        else: # position is bad
                            wheel_velos = self.motors[id].move_to_target(my_posture, [min(wait_x,bx),GOAL_WIDTH-0.1], damping=0)
                    elif (bx<=wait_x+0.07 and by<=-GOAL_WIDTH+0.1): # down coming
                        if (y<=-GOAL_WIDTH+0.1) and (th<=self.cal.d2r(-90) and th>=self.cal.d2r(-100)): # position and angle is good: PUSH!!
                            wheel_velos = self.motors[id].move_to_target(my_posture, [min(wait_x,bx),by], damping=0)
                        elif (y<=-GOAL_WIDTH+0.1): #position is good, but angle is bad: TURN!!
                            wheel_velos = self.motors[id].spin_to_theta(th, self.cal.d2r(-90))
                        else: # position is bad
                            wheel_velos = self.motors[id].move_to_target(my_posture, [min(wait_x,bx),-GOAL_WIDTH+0.1], damping=0)
                    else:
                        cur_trans = self.data_proc.get_cur_ball_transition()
                        NUM_OF_PREDICTED_FRAMES = 1
                        target = [cur_ball[0] + cur_trans[0]*NUM_OF_PREDICTED_FRAMES,
                                  cur_ball[1] + cur_trans[1]*NUM_OF_PREDICTED_FRAMES]
                        wheel_velos = self.motors[id].move_to_target(my_posture, target, damping=0.1)
                else: #outer side than ball
                    wheel_velos = self.motors[id].move_to_target(my_posture, target, damping=0)
            # not contact with ball at wall yet
            else:
                cur_trans = self.data_proc.get_cur_ball_transition()
                NUM_OF_PREDICTED_FRAMES = 1
                target = [cur_ball[0] + cur_trans[0]*NUM_OF_PREDICTED_FRAMES,
                          cur_ball[1] + cur_trans[1]*NUM_OF_PREDICTED_FRAMES]
                wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, [wait_x,target[1],static_theta], damping=0.1)
                #wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, [wait_x,target[1],self.cal.d2r(180)], damping=0.1)

        # ball is coming
        elif (target[0] < wait_x+0.2 and target[1]<GOAL_WIDTH+0.2 and target[1]>-GOAL_WIDTH-0.2): 
            cur_trans = self.data_proc.get_cur_ball_transition()
            NUM_OF_PREDICTED_FRAMES = 1
            target = [cur_ball[0] + cur_trans[0]*NUM_OF_PREDICTED_FRAMES,
                      cur_ball[1] + cur_trans[1]*NUM_OF_PREDICTED_FRAMES]
            wheel_velos = self.motors[id].move_to_target(my_posture, target, damping=0.2)

        # waiting
        else: 
            wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, [wait_x, wait_y, static_theta])

        return wheel_velos
    

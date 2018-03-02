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
        robot_wheels[self.upper_attacker*2:self.upper_attacker*2+2] = self.upper_side_attack(self.upper_attacker)
        robot_wheels[self.lower_attacker*2:self.lower_attacker*2+2] = self.lower_side_attack(self.lower_attacker)
        robot_wheels[self.upper_defender*2:self.upper_defender*2+2] = self.upper_side_defense(self.upper_defender)
        robot_wheels[self.lower_defender*2:self.lower_defender*2+2] = self.lower_side_defense(self.lower_defender)
        robot_wheels[self.goal_keeper*2:self.goal_keeper*2+2] = self.keep_goal(self.goal_keeper)
        return robot_wheels

    def attack(self, id, default_posture):
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
        if bx > -MARGIN: # little margin
            up_pad = FIELD_WIDTH/2 - 0.3
            down_pad = -FIELD_WIDTH/2 + 0.3
            right_pad = FIELD_LENGTH/2 - 0.3
            left_pad = -FIELD_LENGTH/2 + 0.3
            # Corner
            if by > up_pad or by < down_pad or bx > right_pad or bx < left_pad:
                if bx > right_pad: # ball is at opponent's corner
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
                else:
                    if dist_to_ball <= 0.4:
                        if ((static_theta >= math.pi/2 or static_theta <= -math.pi/2) and (y >= by-0.15 and y <= by+0.15) and x > bx):
                            printWrapper("corner blocking")
                            if (y <= 1.5 and y >= 1.2) and (self.cal.r2d(th) >= 180-70 or self.cal.r2d(th) <= -180+70): # Blocking top corner
                                printWrapper("AAA")
                                wheel_velos = [-1.5, -1.8]
                            elif (y >= -1.5 and y <= -1.2) and (self.cal.r2d(th) >= 180-70 or self.cal.r2d(th) <= -180+70    ): # Blocking bottom corner
                                wheel_velos = [-1.8, -1.5]
                                printWrapper("BBB")
                            else:
                                printWrapper("CCC")
                                wheel_velos = self.motors[id].move_to_target(my_posture, cur_ball)
                        else:
                            wheel_velos = self.motors[id].move_to_target(my_posture, cur_ball)

                    else: # Corner, but ball is far
                        if ((static_theta >= math.pi/2 or static_theta <= -math.pi/2) and (y >= by-0.15 and y <= by+0.15) and x > bx): # Ball is in inner area
                            printWrapper("At Corner : Far - INNER")
                            if y >= 0:
                                wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, [-1.8, -1.4])
                            else:
                                wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, [-1.8, +1.4])
                        else:
                            wheel_velos = self.motors[id].move_to_target(my_posture, [bx,by])
                    
            # Inside ground
            else: 
                if ((static_theta >= math.pi/2 or static_theta <= -math.pi/2) and (y >= by-0.15 and y <= by+0.15) and x > bx): # Ball is in inner area
                    printWrapper("At Ground         - INNER")
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
                    else:
                        if y >= 0:
                            wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, [-1.8, +1.4])
                        else:
                            wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, [-1.8, -1.4])
                    wheel_velos = self.motors[id].move_to_target(my_posture, [bx-0.3,by+0.3])
                else: # Ball is in outer area, so PUSH
                    printWrapper("At Ground")
                    ox = 0.2
                    oy = 0.2
                    cur_trans = self.data_proc.get_cur_ball_transition()
                    goal_pstn = [1.1 + 0.75, 0]
                    tar_posture = self.cal.compute_desired_posture(cur_ball, cur_trans, goal_pstn)
                    tar_posture = [tar_posture[0]-0.08,tar_posture[1]-0.08,0]
                    wheel_velos = self.motors[id].move_to_target(my_posture, tar_posture, damping=0)
        else: # DEFEND state
            if id == self.upper_attacker:
                if cur_trans[0] <= -0.015:
                    wheel_velos = self.motors[id].move_to_target(my_posture, [-1.5, PENALTY_AREA_WIDTH/2+0.01])
                else:
                    if bx <= -1.5:
                        if (x >= -1.9-0.05 and x <= -1.8) and (y >= PENALTY_AREA_WIDTH/2-0.1 and y <= PENALTY_AREA_WIDTH/2+0.1): # Arrived at defense position
                            if th >= self.cal.d2r(90-5) and th <= self.cal.d2r(90+5): # Arrived and angle is good, then PUSH
                                if dist_to_ball <= 0.7 and (bx >= -1.8-0.1 and bx <= -1.8+0.1) and by >= PENALTY_AREA_WIDTH/2:
                                    wheel_velos = [1.8, 1.8]
                                else: # Arrivied but ball is far, then WAIT
                                    wheel_velos = [0, 0]
                            else: # Arrived but angle is bad, then TURN
                                wheel_velos = self.motors[id].spin_to_theta(th,self.cal.d2r(90))
                        else:
                            wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, [-1.9, PENALTY_AREA_WIDTH/2+0.01, math.pi/2])
                    else:
                        wheel_velos = self.motors[id].move_to_target(my_posture, cur_ball)
                    
            elif id == self.lower_attacker:
                if cur_trans[0] <= -0.015:
                    wheel_velos = self.motors[id].move_to_target(my_posture, [-1.5, -PENALTY_AREA_WIDTH/2+0.01])
                else:
                    if bx <= -1.5:
                        if (x >= -1.9-0.05 and x <= -1.8) and (y >= -PENALTY_AREA_WIDTH/2-0.1 and y <= -PENALTY_AREA_WIDTH/2+0.1): # Arrived at defense position
                            if th >= self.cal.d2r(-90-5) and th <= self.cal.d2r(-90+5): # Arrived and angle is good, then PUSH
                                if dist_to_ball <= 0.7 and (bx >= -1.8-0.1 and bx <= -1.8+0.1) and by <= -PENALTY_AREA_WIDTH/2:
                                    printWrapper("PUSH!!")
                                    wheel_velos = [1.8, 1.8]
                                else: # Arrivied but ball is far, then WAIT
                                    wheel_velos = [0, 0]
                            else: # Arrived but angle is bad, then TURN
                                wheel_velos = self.motors[id].spin_to_theta(th,self.cal.d2r(-90))
                        else: 
                            wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, [-1.9, -PENALTY_AREA_WIDTH/2-0.01, 3*math.pi/2])
                    else: # In defense state, if ball is in outer area than the robot, follow the ball.
                        wheel_velos = self.motors[id].move_to_target(my_posture, cur_ball)
        return wheel_velos

    def upper_side_attack(self, id):
        default_posture = [0, 0.45, 0]
        return self.attack(id, default_posture)

    def lower_side_attack(self, id):
        default_posture = [0, -0.45, 0]
        return self.attack(id, default_posture)

    def defense(self, id, default_posture):
        our_postures = self.data_proc.get_my_team_postures()
        my_posture = our_postures[id]
        cur_ball = self.data_proc.get_cur_ball_position()
        cur_trans = self.data_proc.get_cur_ball_transition()
        theta_to_ball = self.cal.compute_theta_to_target(my_posture, cur_ball)
        dist_to_ball = self.cal.get_distance(my_posture,cur_ball)
        x = my_posture[0]
        y = my_posture[1]
        th = my_posture[2]
        bx = cur_ball[0]
        by = cur_ball[1]

        if cur_ball[0] < -MARGIN: # DEFENSE state
            ox = 0.05
            cur_trans = self.data_proc.get_cur_ball_transition()
            goal_pstn = [1.1 + 0.75, 0]
            tar_posture = self.cal.compute_desired_posture(cur_ball, cur_trans, goal_pstn)

            if my_posture[0] < cur_ball[0] - ox:
                cur_trans = self.data_proc.get_cur_ball_transition()
                NUM_OF_PREDICTED_FRAMES = 1
                target = [cur_ball[0] + cur_trans[0]*NUM_OF_PREDICTED_FRAMES,
                          cur_ball[1] + cur_trans[1]*NUM_OF_PREDICTED_FRAMES]
                wheel_velos = self.motors[id].move_to_target(my_posture, target, damping=0.2)
            else:
                wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, tar_posture, damping=0.2)
        else: # ATTACK state
            #GAEDOL
            if (bx <= 2.1 and bx >= 1.4 and x >= 1.2) and (by >= -0.4 and by <= 0.4) :
                if dist_to_ball <= 0.15:
                    wheel_velos = self.motors[id].move_to_target(my_posture, cur_ball)
                else:
                    if cur_trans[1] > 0: # ball is coming up
                        if id == self.upper_defender:
                            #wheel_velos = self.motors[id].move_to_target(my_posture, [1.8, 0])
                            cur_trans = self.data_proc.get_cur_ball_transition()
                            NUM_OF_PREDICTED_FRAMES = 1
                            target = [cur_ball[0] + cur_trans[0]*NUM_OF_PREDICTED_FRAMES,
                                      cur_ball[1] + cur_trans[1]*NUM_OF_PREDICTED_FRAMES]
                            wheel_velos = self.motors[id].move_to_target(my_posture, target, damping=0.2)
                        elif id == self.lower_defender:
                            #wheel_velos = self.motors[id].move_to_target(my_posture, [1.8, 0])
                            cur_trans = self.data_proc.get_cur_ball_transition()
                            NUM_OF_PREDICTED_FRAMES = 1
                            target = [cur_ball[0] + cur_trans[0]*NUM_OF_PREDICTED_FRAMES,
                                      cur_ball[1] + cur_trans[1]*NUM_OF_PREDICTED_FRAMES]
                            wheel_velos = self.motors[id].move_to_target(my_posture, target, damping=0.2)
                    else: # ball is coming down
                        if id == self.upper_defender:
                            #wheel_velos = self.motors[id].move_to_target(my_posture, [1.8, 0])
                            cur_trans = self.data_proc.get_cur_ball_transition()
                            NUM_OF_PREDICTED_FRAMES = 1
                            target = [cur_ball[0] + cur_trans[0]*NUM_OF_PREDICTED_FRAMES,
                                      cur_ball[1] + cur_trans[1]*NUM_OF_PREDICTED_FRAMES]
                            wheel_velos = self.motors[id].move_to_target(my_posture, target, damping=0.2)
                        elif id == self.lower_defender:
                            #wheel_velos = self.motors[id].move_to_target(my_posture, [1.8, 0])
                            cur_trans = self.data_proc.get_cur_ball_transition()
                            NUM_OF_PREDICTED_FRAMES = 1
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
                        printWrapper("Backup but ball inner coming")
                        wheel_velos = [-1.8, -1.8]
                        #elif (dist_to_ball <= 0.15) and (cur_trans[1] < 0) # ball is coming up in inner area
                    else: #Not arrived at backup position
                        print("Backup")
                        if id == self.lower_defender:
                            wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, [1.3, -0.15, 0])
                        elif id == self.upper_defender:
                            wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, [1.3, +0.15, 0])
        return wheel_velos

    def upper_side_defense(self, id):
        our_postures = self.data_proc.get_my_team_postures()
        my_posture = our_postures[id]
        cur_ball = self.data_proc.get_cur_ball_position()

        y = max(0.0, cur_ball[1])
        default_posture = [-0.55, y, 0]
        if cur_ball[0] < -FIELD_LENGTH/4 and cur_ball[1] > PENALTY_AREA_WIDTH/2:
            default_posture = [-FIELD_LENGTH / 2, GOAL_WIDTH / 2 + ROBOT_SIZE/2, 0 ]
            wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, default_posture)
            return wheel_velos

        return self.defense(id, default_posture)

    def lower_side_defense(self, id):
        our_postures = self.data_proc.get_my_team_postures()
        my_posture = our_postures[id]
        cur_ball = self.data_proc.get_cur_ball_position()

        y = min(0.0, cur_ball[1])
        default_posture = [-0.55, y, 0]
        if cur_ball[0] < -FIELD_LENGTH/4 and cur_ball[1] < -PENALTY_AREA_WIDTH/2:
            default_posture = [-FIELD_LENGTH / 2, -GOAL_WIDTH / 2 - ROBOT_SIZE/2, 0 ]
            wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, default_posture)
            return wheel_velos

        return self.defense(id, default_posture)

    def keep_goal(self, id):
        our_postures = self.data_proc.get_my_team_postures()
        my_posture = our_postures[id]
        cur_ball = self.data_proc.get_cur_ball_position()
        theta_to_ball = self.cal.compute_theta_to_target(my_posture, cur_ball)
        static_theta = self.cal.compute_static_theta(my_posture, cur_ball)

        cur_trans = self.data_proc.get_cur_ball_transition()
        NUM_OF_PREDICTED_FRAMES = 3
        target = [cur_ball[0] + cur_trans[0]*NUM_OF_PREDICTED_FRAMES,
                  cur_ball[1] + cur_trans[1]*NUM_OF_PREDICTED_FRAMES]
        predict_ball_theta = self.cal.compute_static_theta(cur_ball, target)

        x = my_posture[0]
        y = my_posture[1]
        th = my_posture[2]
        bx = cur_ball[0]
        by = cur_ball[1]

        wait_x = -1.85
        wait_y = max(min(target[1], GOAL_WIDTH-0.1), -GOAL_WIDTH + 0.1)


        if (bx < wait_x+0.3 and by<GOAL_WIDTH+0.2 and by>-GOAL_WIDTH-0.2): # ball is coming
            #if (x<wait_x and y<wait_y)
            printWrapper("Goalkeeper : Ball is coming")
            cur_trans = self.data_proc.get_cur_ball_transition()
            NUM_OF_PREDICTED_FRAMES = 1
            target = [cur_ball[0] + cur_trans[0]*NUM_OF_PREDICTED_FRAMES,
                      cur_ball[1] + cur_trans[1]*NUM_OF_PREDICTED_FRAMES]
            wheel_velos = self.motors[id].move_to_target(my_posture, target, damping=0.2)
        else: #wait
            printWrapper("Goalkeeper : Waiting")
            wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, [wait_x, wait_y, static_theta])

        return wheel_velos
    

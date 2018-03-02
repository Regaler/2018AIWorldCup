from __future__ import division
from __future__ import print_function
import math
from motor import *
from calculator import *
from data_processor import *
import sys

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
        our_postures = self.data_proc.get_my_team_postures()
        my_posture = our_postures[id]
        cur_ball = self.data_proc.get_cur_ball_position()
        cur_trans = self.data_proc.get_cur_ball_transition()
        theta_to_ball = self.cal.compute_theta_to_target(my_posture, cur_ball)
        dist_to_ball = self.cal.get_distance(my_posture,cur_ball)
        x = my_posture[0]
        y = my_posture[1]
        th = my_posture[2]
        if id == 0:
            printWrapper("0th coordinate is: " + str(my_posture))
        # ATTACK state
        if cur_ball[0] > -MARGIN: # little margin
            up_pad = FIELD_WIDTH/2 - 0.3
            down_pad = -FIELD_WIDTH/2 + 0.3
            right_pad = FIELD_LENGTH/2 - 0.3
            left_pad = -FIELD_LENGTH/2 + 0.3
            # Corner
            if cur_ball[1] > up_pad or cur_ball[1] < down_pad or cur_ball[0] > right_pad or cur_ball[0] < left_pad:
                """
                if y >= 0: # It's top corner
                    if (x <= FIELD_LENGTH/2 and x >= FIELD_LENGTH/2-BALL_RADIUS*2-GOAL_DEPTH) and (self.cal.r2d(th) >= 80 and self.cal.r2d(th) <= 100): # Blocking top right corner
                        wheel_velos = [-1.5, -1.8]
                    elif (y <= FIELD_WIDTH/2 and y >= FIELD_WIDTH/2 + ROBOT_SIZE*2) and (self.cal.r2d(th) >= 180-40 and self.cal.r2d(th) <= 180+40): # Blocking top corner
                        wheel_velos = [-1.5, -1.8]
                else: # It's blocking bottom corner
                    if (x <= FIELD_LENGTH/2 and x >= FIELD_LENGTH/2-BALL_RADIUS*2-GOAL_DEPTH) and (self.cal.r2d(th) >= 80+180 and self.cal.r2d(th) <= 100+180): # Blocking bottom right corner
                        wheel_velos = [-1.8, -1.5]
                    elif (y >= -FIELD_WIDTH/2 and y <= -FIELD_WIDTH/2 + ROBOT_SIZE*2) and (self.cal.r2d(th) >= 180-40 and self.cal.r2d(th) <= 180+40): # Blocking bottom corner
                        wheel_velos = [-1.8, -1.5]
                """
                if dist_to_ball <= 0.15: 
                    if (y>=0) and (x <= 2.1 and x >= 1.7) and (self.cal.r2d(th) >= 80 and self.cal.r2d(th) <= 100): # Blocking top right corner
                        printWrapper("AAA")
                        wheel_velos = [-1.5, -1.8]
                    elif (y>=0) and (y <= 1.5 and y >= 1.2) and (self.cal.r2d(th) >= 180-40 and self.cal.r2d(th) <= 180+40): # Blocking top corner
                        wheel_velos = [-1.5, -1.8]
                        printWrapper("BBB")
                    elif (y<0) and (x <= 2.1 and x >= 1.7) and (self.cal.r2d(th) >= 80+180 and self.cal.r2d(th) <= 100+180): # Blocking bottom right corner
                        wheel_velos = [-1.8, -1.5]
                        printWrapper("CCC")
                    elif (y<0) and (y >= -1.5 and y <= -1.2) and (self.cal.r2d(th) >= 180-40 and self.cal.r2d(th) <= 180+40): # Blocking bottom corner
                        wheel_velos = [-1.8, -1.5]
                        printWrapper("EEE")
                    else:
                        wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, [cur_ball[0],cur_ball[1],my_posture[2]], damping=0)
                else:
                    wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, [cur_ball[0],cur_ball[1],my_posture[2]], damping=0)
                return wheel_velos
            # Inside ground
            ox = 0.2
            oy = 0.2
            cur_trans = self.data_proc.get_cur_ball_transition()
            goal_pstn = [1.1 + 0.75, 0]
            tar_posture = self.cal.compute_desired_posture(cur_ball, cur_trans, goal_pstn)
            tar_posture = [tar_posture[0]-0.05,tar_posture[1]-0.05,0]
            wheel_velos = self.motors[id].move_to_target(my_posture, tar_posture, damping=0)
            """
            if self.cal.is_desired_posture(my_posture, tar_posture) == True:
                wheel_velos = self.motors[id].move_to_target(my_posture, goal_pstn, damping=0)
            else:
                wheel_velos = self.motors[id].move_to_target(my_posture, tar_posture, damping=0)
            """
        else: # DEFEND state
            #wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, default_posture)
            if id == self.upper_attacker:
                if cur_trans[0] <= -0.015:
                    wheel_velos = self.motors[id].move_to_target(my_posture, [-FIELD_LENGTH/2, PENALTY_AREA_WIDTH/2+0.01])
                else:
                    if cur_ball[0] <= -FIELD_LENGTH/2+PENALTY_AREA_DEPTH/2+GOAL_DEPTH:
                        wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, [-FIELD_LENGTH/2, PENALTY_AREA_WIDTH/2+0.01, 0.9*math.pi/2])
                    else:
                        wheel_velos = self.motors[id].move_to_target(my_posture, cur_ball)
                    
            elif id == self.lower_attacker:
                if math.sqrt(cur_trans[0]*cur_trans[0]+cur_trans[1]*cur_trans[1]) >= 0.05:
                    wheel_velos = self.motors[id].move_to_target(my_posture, [-FIELD_LENGTH/2, -PENALTY_AREA_WIDTH/2+0.01])
                else:
                    if cur_ball[0] <= -FIELD_LENGTH/2+PENALTY_AREA_DEPTH/2+GOAL_DEPTH:
                        wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, [-FIELD_LENGTH/2, -PENALTY_AREA_WIDTH/2+0.01, 0.9*math.pi/2])
                    else:
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
        dist_to_ball = self.cal.get_distance(my_posture, cur_ball)

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
            #wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, default_posture)
            if dist_to_ball <= 0.3 and cur_ball[0] >= FIELD_LENGTH/3:
                wheel_velos = self.motors[id].move_to_target(my_posture,cur_ball)
            elif (cur_ball[0] <= FIELD_LENGTH/2 and cur_ball[0] >= FIELD_LENGTH/2-0.1) and (cur_ball[1] >= -FIELD_WIDTH/2 and cur_ball[1] <= FIELD_WIDTH/2):
                wheel_velos = self.motors[id].move_to_target(my_posture,cur_ball)
            else:
                if id == self.lower_defender:
                    wheel_velos = self.motors[id].move_to_target(my_posture,[FIELD_LENGTH/4, FIELD_WIDTH/4])
                elif id == self.upper_defender:
                    wheel_velos = self.motors[id].move_to_target(my_posture,[FIELD_LENGTH/4, -FIELD_WIDTH/4])

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
        printWrapper("goal keeper is at: " + str(my_posture))
        x = - FIELD_LENGTH/2
        y = max(min(cur_ball[1], GOAL_WIDTH/2 - ROBOT_SIZE/4), -GOAL_WIDTH/2 + ROBOT_SIZE/4)

        if cur_ball[0] < -FIELD_LENGTH/2 + PENALTY_AREA_DEPTH and abs(cur_ball[1] < GOAL_WIDTH/2):
            cur_trans = self.data_proc.get_cur_ball_transition()
            NUM_OF_PREDICTED_FRAMES = 1
            target = [cur_ball[0] + cur_trans[0]*NUM_OF_PREDICTED_FRAMES,
                      cur_ball[1] + cur_trans[1]*NUM_OF_PREDICTED_FRAMES]
            wheel_velos = self.motors[id].move_to_target(my_posture, target, damping=0.2)
        else:
            wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, [x, y, 0])

        return wheel_velos
    

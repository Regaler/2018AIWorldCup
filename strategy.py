import math
from motor import *
from calculator import *
from data_processor import *

class Strategy(object):
    def __init__(self, data_proc, is_debug=False):
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

        if cur_ball[0] > -0.1: # little margin
            up_pad = FIELD_WIDTH/2 - 0.3
            down_pad = -FIELD_WIDTH/2 + 0.3
            right_pad = FIELD_LENGTH/2 - 0.3
            left_pad = -FIELD_LENGTH/2 + 0.3

            if cur_ball[1] > up_pad or cur_ball[1] < down_pad or cur_ball[0] > right_pad or cur_ball[0] < left_pad:
                wheel_velos = self.motors[id].move_to_target(my_posture, cur_ball, damping=0.35)
                return wheel_velos

            ox = 0.2
            oy = 0.2
            cur_trans = self.data_proc.get_cur_ball_transition()
            goal_pstn = [1.1 + 0.75, 0]
            tar_posture = self.cal.compute_desired_posture(cur_ball, cur_trans, goal_pstn)
            if self.cal.is_desired_posture(my_posture, tar_posture) == True:
                wheel_velos = self.motors[id].move_to_target(my_posture, goal_pstn, damping=0.2)
            else:
                wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, tar_posture, damping=0.2)
        else:
            wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, default_posture)

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

        if cur_ball[0] < -0.1: # little margin
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
        else:
            wheel_velos = self.motors[id].three_phase_move_to_target(my_posture, default_posture)

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
    
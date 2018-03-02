import math
from motor import *
from calculator import *
from data_processor import *

def printWrapper2(msg):
    print(msg)
    sys.__stdout__.flush() # stdout is redirected to somewhere else. flush __stdout__ directly.

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

class Strategy2(object):
    def __init__(self, data_proc, is_debug=False):
        printWrapper2("Strategy2: __init__")
        self.data_proc = data_proc
        self.cal = Calculator(is_debug=False)
        # raw_state looks like: [(x1,y1,th1),..,(x5,y5,th5),(x_ball,y_ball)]
        self.motors = []
        self.motors.append(Motor('Motor 0', is_debug=False))
        self.motors.append(Motor('Motor 1', is_debug=False))
        self.motors.append(Motor('Motor 2', is_debug=False))
        self.motors.append(Motor('Motor 3', is_debug=False))
        self.motors.append(Motor('Motor 4', is_debug=False))

    def perform(self):
        printWrapper2("Strategy2: perform")
        # Process to get robot_wheels
        state = self.raw_state_to_state()
        printWrapper2("perform: state: " + str(state))
        roles = self.get_roles(state)
        printWrapper2("perform: roles: " + str(roles))
        actions = self.policy(state, roles)
        #printWrapper2("perform: actions: " + str(actions))
        robot_wheels = self.actions_to_raw_action(actions)
        return robot_wheels

    def raw_state_to_state(self):
        #printWrapper2("Strategy2: raw_state_to_state")
        """
        Input: raw_state
        Output: state (conceptual 'situational' state)
        Example: (x1,y1,th1,x2,...,x10,y10,th10,ball_x,ball_y,ball_th) --> 'ATTACK_CHANCE' state
        """
        ball_pos = self.data_proc.get_cur_ball_position()
        ball_x = ball_pos[0]
        ball_y = ball_pos[1]

        if FIELD_LENGTH/2-PENALTY_AREA_DEPTH < ball_x and -PENALTY_AREA_WIDTH/2 < ball_y and ball_y < PENALTY_AREA_WIDTH/2: # R1
            return 'ATTACK_CHANCE'
        elif FIELD_LENGTH/3-2*PENALTY_AREA_DEPTH/3 < ball_x and -FIELD_WIDTH/4-PENALTY_AREA_WIDTH/4 < ball_y and ball_y < FIELD_WIDTH/4 + PENALTY_AREA_WIDTH/4: # R2
            return 'ATTACK_HARD'
        elif FIELD_LENGTH/6-PENALTY_AREA_DEPTH/3 < ball_x: # R3
            return 'ATTACK_SOFT'
        elif -(FIELD_LENGTH/6 - PENALTY_AREA_DEPTH/3) < ball_x: # R4
            return 'NORMAL'
        elif -(FIELD_LENGTH/3 - 2*PENALTY_AREA_DEPTH/3) < ball_x and (FIELD_WIDTH/4 + PENALTY_AREA_WIDTH/4 < ball_y or ball_y < -(FIELD_WIDTH/4 + PENALTY_AREA_WIDTH/4)): # R5
            return 'DEFENSE_SOFT'
        elif FIELD_LENGTH/2 - PENALTY_AREA_DEPTH < ball_x and (PENALTY_AREA_WIDTH/2 < ball_y or ball_y < -PENALTY_AREA_WIDTH/2):
            return 'DEFENSE_HARD'
        else:
            return 'DEFENSE_EMERGENCY'

    def get_roles(self, state):
        #printWrapper2("Strategy2: get_roles")
        our_postures = self.data_proc.get_my_team_postures()
        roles = []
        x = [our_postures[0][0],our_postures[1][0],our_postures[2][0],our_postures[3][0],our_postures[4][0]]

        if state == 'ATTACK_CHANCE':
            roles = ['GOAL_KEEPER','ATTACKER','ATTACKER','ATTACKER','ATTACKER']
            x, roles = zip(*sorted(zip(x, roles)))
        elif state == 'ATTACK_HARD':
            roles = ['GOAL_KEEPER','ATTACKER','ATTACKER','ATTACKER','ATTACKER']
            x, roles = zip(*sorted(zip(x, roles)))
        elif state == 'ATTACK_SOFT':
            roles = ['GOAL_KEEPER','DEFENDER','ATTACKER','ATTACKER','ATTACKER']
            x, roles = zip(*sorted(zip(x, roles)))
        elif state == 'NORMAL':
            roles = ['GOAL_KEEPER','DEFENDER','DEFENDER','ATTACKER','ATTACKER']
            x, roles = zip(*sorted(zip(x, roles)))
        elif state == 'DEFENSE_SOFT':
            roles = ['GOAL_KEEPER','DEFENDER','DEFENDER','DEFENDER','ATTACKER']
            x, roles = zip(*sorted(zip(x, roles)))
        elif state == 'DEFENSE_HARD':
            roles = ['GOAL_KEEPER','DEFENDER','DEFENDER','DEFENDER','DEFENDER']
            x, roles = zip(*sorted(zip(x, roles)))
        elif state == 'DEFENSE_EMERGENCY':
            roles = ['GOAL_KEEPER','DEFENDER','DEFENDER','DEFENDER','DEFENDER']
            x, roles = zip(*sorted(zip(x, roles)))
        else:
            raise ValueError('get_roles: No such state')
        return roles

    def policy(self, state, roles):
        #printWrapper2("Strategy2: policy")
        """
        Input: state (conceptual 'situational' state)
        Output: actions
        """
        actions = []
        if state == 'ATTACK_CHANCE':
            for player in roles:
                if player == 'GOAL_KEEPER':
                    action = 'GOAL_KEEP'
                    actions.append(action)
                elif player == 'ATTACKER':
                    action = 'ATTACK_RUSH'
                    actions.append(action)
                elif player == 'DEFENDER':
                    action = 'ATTACK_RUSH'
                    actions.append(action)
                else:
                    raise ValueError('policy: No such role.')
        elif state == 'ATTACK_HARD':
            for player in roles:
                if player == 'GOAL_KEEPER':
                    action = 'GOAL_KEEP'
                    actions.append(action)
                elif player == 'ATTACKER':
                    action = 'ATTACK_RUSH'
                    actions.append(action)
                elif player == 'DEFENDER':
                    action = 'ATTACK_KICK'
                    actions.append(action)
                else:
                    raise ValueError('policy: No such role.')
        elif state == 'ATTACK_SOFT':
            for player in roles:
                if player == 'GOAL_KEEPER':
                    action = 'GOAL_KEEP'
                    actions.append(action)
                elif player == 'ATTACKER':
                    action = 'ATTACK_KICK'
                    actions.append(action)
                elif player == 'DEFENDER':
                    action = 'ATTACK_KICK'
                    actions.append(action)
                else:
                    raise ValueError('policy: No such role.')
        elif state == 'NORMAL':
            for player in roles:
                if player == 'GOAL_KEEPER':
                    action = 'GOAL_KEEP'
                    actions.append(action)
                elif player == 'ATTACKER':
                    action = 'ATTACK_KICK'
                    actions.append(action)
                elif player == 'DEFENDER':
                    action = 'DEFEND_READY'
                    actions.append(action)
                else:
                    raise ValueError('policy: No such role.')
        elif state == 'DEFENSE_SOFT':
            for player in roles:
                if player == 'GOAL_KEEPER':
                    action = 'GOAL_KEEP'
                    actions.append(action)
                elif player == 'ATTACKER':
                    action = 'DEFEND_READY'
                    actions.append(action)
                elif player == 'DEFENDER':
                    action = 'DEFEND_KICK'
                    actions.append(action)
                else:
                    raise ValueError('policy: No such role.')
        elif state == 'DEFENSE_HARD':
            for player in roles:
                if player == 'GOAL_KEEPER':
                    action = 'GOAL_KEEP'
                    actions.append(action)
                elif player == 'ATTACKER':
                    action = 'DEFEND_UPPER_CORNER'
                    actions.append(action)
                elif player == 'DEFENDER':
                    action = 'DEFEND_LOWER_CORNER'
                    actions.append(action)
                else:
                    raise ValueError('policy: No such role.')
        elif state == 'DEFENSE_EMERGENCY':
            for player in roles:
                if player == 'GOAL_KEEPER':
                    action = 'GOAL_KEEP'
                    actions.append(action)
                elif player == 'ATTACKER':
                    action = 'DEFEND_KICK'
                    actions.append(action)
                elif player == 'DEFENDER':
                    action = 'GOAL_KEEP'
                    actions.append(action)
                else:
                    raise ValueError('policy: No such role.')
        else:
            raise ValueError('policy: No such state.')
        return actions

    def actions_to_raw_action(self, actions):
        #printWrapper2("Strategy2: actions_to_raw_action")
        """
        Input: action
        Output: raw_action
        Ex: 'CHASE_BALL' -> ()
        """
        raw_state = self.data_proc.get_my_team_postures()
        cur_ball = self.data_proc.get_cur_ball_position()
        raw_actions = []
        for ID in range(len(actions)):
            # Initialization
            my_posture = raw_state[ID]
            relative = []

            # relative position to ball: out_close, out_far, in_close, in_far
            dist_to_ball = self.cal.get_distance(my_posture, cur_ball)
            angle_to_ball = self.cal.compute_static_theta(my_posture, cur_ball)
            if cur_ball[1] >= 0:
                ball_angle = self.cal.compute_static_theta(cur_ball, [FIELD_WIDTH/2, GOAL_WIDTH/2])
            else:    
                ball_angle = self.cal.compute_static_theta(cur_ball, [FIELD_WIDTH/2, -GOAL_WIDTH/2])
            # out or in
            if my_posture[0] < cur_ball[0]:
                relative.append("out")
            else:
                relative.append("in")
            # close or far
            if dist_to_ball <= 0.1:
                relative.append("close")
            else:
                relative.append("far")
            # rush or move
            if abs(ball_angle) <= angle_to_ball:
                relative.append("rush")
            else:
                relative.append("move")

            # transforms high level action into wheels speed
            if actions[ID] == 'GOAL_KEEP':
                x = - FIELD_LENGTH/2
                y = max(min(cur_ball[1], GOAL_WIDTH/2 - ROBOT_SIZE/4), -GOAL_WIDTH/2 + ROBOT_SIZE/4)
                if "out" in relative:  
                    if cur_ball[0] < -FIELD_LENGTH/2 + PENALTY_AREA_DEPTH and abs(cur_ball[1] < GOAL_WIDTH/2):
                        cur_trans = self.data_proc.get_cur_ball_transition()
                        NUM_OF_PREDICTED_FRAMES = 1
                        target = [cur_ball[0] + cur_trans[0]*NUM_OF_PREDICTED_FRAMES,
                                  cur_ball[1] + cur_trans[1]*NUM_OF_PREDICTED_FRAMES]
                        raw_actions += self.motors[ID].move_to_target(my_posture, target, damping=0.2)
                    else:
                        raw_actions += self.motors[ID].three_phase_move_to_target(my_posture, [x, y, 0])
                elif "in" in relative:
                    raw_actions += self.motors[ID].move_to_target(my_posture, [x,y])
                
            elif actions[ID] == 'DEFEND_UPPER_CORNER':
                """
                if "out" in relative:
                    if "close" in relative:
                        if "rush" in relative:
                            raw_actions += self.rush_to_goalpost(ID,my_posture)
                        else: # out, close, but angle is bad
                            raw_actions += self.move_to_ball(ID,my_posture,cur_ball)
                    else "far" in relative:
                        raw_actions += self.move_to_ball(ID,my_posture,cur_ball)
                else: # in
                    raw_actions += self.move_to_ball(ID,my_posture,cur_ball)
                """
                raw_actions += self.motors[ID].move_to_target(my_posture, cur_ball)
            elif actions[ID] == 'DEFEND_LOWER_CORNER':
                """
                if "out" in relative:
                    if "close" in relative:
                        if "rush" in relative:
                            raw_actions += self.rush_to_goalpost(ID,my_posture)
                        else: # out, close, but angle is bad
                            raw_actions += self.move_to_ball(ID,my_posture,cur_ball)
                    else "far" in relative:
                        raw_actions += self.move_to_ball(ID,my_posture,cur_ball)
                else: # in
                    raw_actions += self.move_to_ball(ID,my_posture,cur_ball)
                """
                raw_actions += self.motors[ID].move_to_target(my_posture, cur_ball)
            elif actions[ID] == 'DEFEND_KICK':
                """
                if "out" in relative:
                    if "close" in relative:
                        if "rush" in relative:
                            raw_actions += self.rush_to_goalpost(ID,my_posture)
                        else: # out, close, but angle is bad
                            raw_actions += self.move_to_ball(ID,my_posture,cur_ball)
                    else "far" in relative:
                        raw_actions += self.move_to_ball(ID,my_posture,cur_ball)
                else: # in
                    raw_actions += self.move_to_ball(ID,my_posture,cur_ball)
                """
                raw_actions += self.motors[ID].move_to_target(my_posture, cur_ball)
            elif actions[ID] == 'ATTACK_KICK':
                #printWrapper2("ATTACK_KICK")
                if "out" in relative:
                    if "close" in relative:
                        if "rush" in relative:
                            #raw_actions += self.motors[ID].move_to_target(my_posture, cur_ball)
                            raw_actions += self.rush_to_goalpost(ID, my_posture)
                        else: # out, close, but angle is bad
                            raw_actions += self.rush_to_goalpost(ID, my_posture)
                    else:
                        raw_actions += self.move_to_ball(ID, my_posture)
                else: # in
                    if "close" in relative:
                        raw_actions += self.jitter(ID, my_posture)
                    else:
                        raw_actions += self.move_to_ball(ID, my_posture)
            elif actions[ID] == 'DEFEND_READY':
                """
                if "out" in relative:
                    if "close" in relative:
                        if "rush" in relative:
                            raw_actions += self.rush_to_goalpost(ID,my_posture)
                        else: # out, close, but angle is bad
                            raw_actions += self.move_to_ball(ID,my_posture,cur_ball)
                    else "far" in relative:
                        raw_actions += self.move_to_ball(ID,my_posture,cur_ball)
                else: # in
                    raw_actions += self.move_to_ball(ID,my_posture,cur_ball)
                """
                raw_actions += self.motors[ID].move_to_target(my_posture, cur_ball)
            elif actions[ID] == 'ATTACK_RUSH':
                #printWrapper2("ATTACK_RUSH")
                if "out" in relative:
                    if "close" in relative:
                        if "rush" in relative:
                            #raw_actions += self.motors[ID].move_to_target(my_posture, cur_ball)
                            raw_actions += self.rush_to_goalpost(ID, my_posture)
                        else: # out, close, but angle is bad
                            raw_actions += self.rush_to_goalpost(ID, my_posture)
                    else:
                        raw_actions += self.move_to_ball(ID, my_posture)
                else: # in
                    if "close" in relative:
                        raw_actions += self.jitter(ID, my_posture)
                    else:
                        raw_actions += self.move_to_ball(ID, my_posture)
                raw_actions += self.motors[ID].move_to_target(my_posture, cur_ball)
        return raw_actions

    """
    Some high level motion functions. They all use move_to_target function.
    """
    
    def move_to_ball(self, ID, my_posture):
        cur_ball = self.data_proc.get_cur_ball_position()
        return self.motors[ID].move_to_target(my_posture, [cur_ball[0]-0.15, cur_ball[1]])

    def rush_to_goalpost(self, ID, my_posture):
        return self.motors[ID].move_to_target(my_posture, [FIELD_LENGTH/2,0])

    def jitter(self, ID, my_posture):
        return self.motors[ID].move_to_target(my_posture, [0,0])
    
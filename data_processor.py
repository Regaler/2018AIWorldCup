NUMBER_OF_ROBOTS = 5
FIELD_LENGTH = 2.2
FIELD_WIDTH  = 1.8
GOAL_DEPTH   = 0.15
GOAL_WIDTH   = 0.4
PENALTY_AREA_DEPTH = 0.35
PENALTY_AREA_WIDTH = 0.8

BALL_RADIUS = 0.02135
ROBOT_SIZE = 0.075
AXLE_LENGTH = 0.07
WHEEL_RADIUS = 0.03

ATTACK_MARGIN = 0.9
DEFENSE_MARGIN = 0.1

class Data_processor(object):
    def __init__(self, is_debug=False):
        self.is_debug = is_debug
        self.cur_frame = None
        self.prev_frame = None
        if self.is_debug:
            print('Data processor constructed')

    def update_cur_frame(self, frame):
        if self.cur_frame is not None:
            self.prev_frame = self.cur_frame
        self.cur_frame = frame

    def get_cur_postures(self, team):
        cur_postures = []
        for id in range(NUMBER_OF_ROBOTS):
            posture = [0]*3
            posture[0] = self.cur_frame.coordinates[team][id][0]
            posture[1] = self.cur_frame.coordinates[team][id][1]
            posture[2] = self.cur_frame.coordinates[team][id][2]
            cur_postures.append(posture)
        return cur_postures

    def get_my_team_postures(self):
        return self.get_cur_postures(0)

    def get_opponent_postures(self):
        return self.get_cur_postures(1)

    def get_cur_ball_position(self):
        cur_ball = [self.cur_frame.coordinates[2][0], self.cur_frame.coordinates[2][1]]
        return cur_ball

    def get_ball_position(self, frame):
        ball_ptsn = [frame.coordinates[2][0], frame.coordinates[2][1]]
        return ball_ptsn

    def get_cur_ball_transition(self):
        if self.cur_frame is None:
            transition = [0, 0]
        else:
            cur_ball = self.get_ball_position(self.cur_frame)
            prev_ball = self.get_ball_position(self.prev_frame)
            transition = [cur_ball[0] - prev_ball[0], cur_ball[1] - prev_ball[1]]

        #return [0, 0]
        return transition

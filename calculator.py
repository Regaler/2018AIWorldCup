import math
import sys

TH_TOLERANCE = 0.1
DIST_TOLERANCE = 0.05

class Calculator(object):
    def __init__(self, is_debug=False):
        self.is_debug = is_debug
        if self.is_debug:
            print('Calculator contructed')

    def d2r(self, deg):
        return deg*math.pi/180

    def r2d(self, rad):
        return rad*180/math.pi

    def get_distance(self, pointA, pointB):
        dx = pointA[0] - pointB[0]
        dy = pointA[1] - pointB[1]
        return math.sqrt(dx*dx + dy*dy)

    def theta_equal_with_tolerance(self, cur_th, tar_th, tolerance):
        d_th = cur_th - tar_th
        while d_th > math.pi:
            d_th -= 2*math.pi
        while d_th < -math.pi:
            d_th += 2*math.pi
        if self.is_debug:
            print('%s: (d_th, tolerance) = (%.4f, %.4f)' %(sys._getframe().f_code.co_name, d_th, tolerance))

        if abs(d_th) < tolerance:
            return True
        return False

    def compute_static_theta(self, current, target):
        """
        args:
        - current, target: current, target position (x, y)
        return: theta in rad
        """
        dx = target[0] - current[0]
        dy = target[1] - current[1]

        if dx == 0 and dy == 0:
            static_th = 0
        elif dx == 0 and dy > 0:
            static_th = math.pi/2
        elif dx == 0 and dy < 0:
            static_th = -math.pi/2
        else:
            static_th = math.atan2(dy, dx)

        return static_th

    def compute_theta_to_target(self, cur_posture, target):
        """
        args:
        - cur_posture: (x, y, th)
        - target: (x, y)
        return: theta in rad
        """
        current = cur_posture[0:2]
        static_th = self.compute_static_theta(current, target)
        d_th = static_th - cur_posture[2]
        while d_th > math.pi:
            d_th -= 2*math.pi
        while d_th < -math.pi:
            d_th += 2*math.pi
        if self.is_debug:
            print('%s: (static theta, target theta) = (%.4f, %.4f)' %(sys._getframe().f_code.co_name, static_th, d_th))

        return d_th

    def compute_desired_posture(self, ball_pstn, cur_trans, goal_pstn):
        NUM_OF_PREDICTED_FRAMES = 1
        pad = 0.1 # player behind the ball with pad
        static_th = self.compute_static_theta(ball_pstn, goal_pstn)
        x = ball_pstn[0] - pad*math.cos(static_th) + cur_trans[0]*NUM_OF_PREDICTED_FRAMES
        y = ball_pstn[1] - pad*math.sin(static_th) + cur_trans[1]*NUM_OF_PREDICTED_FRAMES
        th = static_th
        return [x, y, th]

    def is_desired_posture(self, cur_posture, tar_posture):
        if self.get_distance(cur_posture, tar_posture) < DIST_TOLERANCE and self.theta_equal_with_tolerance(cur_posture[2], tar_posture[2], TH_TOLERANCE) == True:
            return True
        return False


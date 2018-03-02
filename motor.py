from pid_ctl import *
from calculator import *
import sys
import math

class Motor(object):
    def __init__(self, name, is_debug=False):
        self.spin_ctrl = Pid_ctl(0.25, 0.005, 0.01, 0.05, is_debug=False)
        self.cal = Calculator(is_debug=False)
        self.cur_phase = 0
        self.is_debug = is_debug
        self.name = name
        if self.is_debug:
            print('%s constructed' %self.name)

    def spin_to_theta(self, cur_th, tar_th):
        """
        spin the robot to theta, pid controller is used to control the velocity

        args:
        - cur_th: current theta in radian
        - tar_th: target theta in radian

        return: linear velocity of left and right wheels
        """
        err = tar_th - cur_th
        while err > math.pi:
            err -= 2*math.pi
        while err < -math.pi:
            err += 2*math.pi

        velo = self.spin_ctrl.control(err)
        wheel_velos = [-velo, velo]
        return wheel_velos

    def move_to_target(self, cur_posture, target, damping=0.2):
        """
        move the robot to target

        args:
        - cur_posture: current x, y, theta
        - target: target x, y

        return: linear velocity of left and right wheels
        """
        mult_lin = 2
        mult_ang = 0.2
        d_e = self.cal.get_distance(cur_posture, target)
        d_th = self.cal.compute_theta_to_target(cur_posture, target)

        if d_e > 1:
            ka = 17
        elif d_e > 0.5:
            ka = 19
        elif d_e > 0.3:
            ka = 21
        elif d_e > 0.2:
            ka = 23
        else:
            ka = 25
        ka /= 90

        sign = 1

        if d_th > self.cal.d2r(95):
            d_th -= math.pi
            sign = -1
        elif d_th < self.cal.d2r(-95):
            d_th += math.pi
            sign = -1

        if abs(d_th) > self.cal.d2r(85):
            wheel_velos = [-mult_ang*d_th, mult_ang*d_th]
        else:
            if d_e < 5.0 and abs(d_th) < self.cal.d2r(40):
                ka = 0.1
            ka *= 4
            left_velo  = sign*(mult_lin*(1/(1 + math.exp(-3*d_e)) - damping) - mult_ang*ka*d_th)
            right_velo = sign*(mult_lin*(1/(1 + math.exp(-3*d_e)) - damping) + mult_ang*ka*d_th)
            wheel_velos = [left_velo, right_velo]

        return wheel_velos

    def three_phase_move_to_target(self, cur_posture, tar_posture, damping=0.2):
        """
        Move to target (x, y, th) with three phase (spin, move, spin)
        An FSM is used with this->cur_phase being the current state
        """
        wheel_velos = [0, 0]

        if self.is_debug:
            print('%s: %s: current posture = (%.4f, %.4f, %.4f)' %(sys._getframe().f_code.co_name,
                self.name, cur_posture[0], cur_posture[1], cur_posture[2]))
            print('%s: %s: target posture = (%.4f, %.4f, %.4f)' %(sys._getframe().f_code.co_name,
                self.name, tar_posture[0], tar_posture[1], tar_posture[2]))

        target  = tar_posture[0:2]
        current = cur_posture[0:2]
        th_to_target = self.cal.compute_theta_to_target(cur_posture, target)

        reset_th = math.pi/3
        reset_dist = 0.3
        if abs(th_to_target) > reset_th and self.cal.get_distance(current, target) > reset_dist:
            if self.is_debug:
                print('%s: Phase is reset' %(sys._getframe().f_code.co_name))
            self.cur_phase = 0

        # FSM for 3-phase movement
        if self.cur_phase == 0:
            # return if distance is too small
            if self.cal.get_distance(current, target) <= DIST_TOLERANCE:
                self.cur_phase = 2
                return [0, 0]

            static_th = self.cal.compute_static_theta(current, target)

            if self.is_debug:
                print('%s: %s: Phase 0, tar theta = %.4f' %(sys._getframe().f_code.co_name, self.name, static_th))

            cur_th = cur_posture[2]
            # spin if current thta is diffent from target theta else switch to phase 1
            if self.cal.theta_equal_with_tolerance(cur_th, static_th, TH_TOLERANCE) == False:
                if self.is_debug:
                    print('%s: %s: Phase 0, spin' %(sys._getframe().f_code.co_name, self.name))
                wheel_velos = self.spin_to_theta(cur_th, static_th)
            else:
                if self.is_debug:
                    print('%s: %s: Move to phase 1' %(sys._getframe().f_code.co_name, self.name))
                self.cur_phase = 1

        if self.cur_phase == 1:
            dist = self.cal.get_distance(current, target)

            if self.is_debug:
                print('%s: %s: Phase 1, distance = %.4f' %(sys._getframe().f_code.co_name, self.name, dist))

            if dist > DIST_TOLERANCE:
                if self.is_debug:
                    print('%s: %s: Phase 1, move to target' %(sys._getframe().f_code.co_name, self.name))
                wheel_velos = self.move_to_target(cur_posture, target, damping)
            else:
                if self.is_debug:
                    print('%s: %s: move to phase 2' %(sys._getframe().f_code.co_name, self.name))
                self.cur_phase = 2

        if self.cur_phase == 2:
            cur_th = cur_posture[2]
            tar_th = tar_posture[2]
            if self.cal.theta_equal_with_tolerance(cur_th, tar_th, TH_TOLERANCE) == False:
                if self.is_debug:
                    print('%s: %s: Phase 2, spin' %(sys._getframe().f_code.co_name, self.name))
                wheel_velos = self.spin_to_theta(cur_th, tar_th)
            else:
                self.cur_phase = 0
                if self.is_debug:
                    print('%s: %s: Finish 3 phase movement, reset' %(sys._getframe().f_code.co_name, self.name))

        return wheel_velos


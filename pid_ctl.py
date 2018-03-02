import sys

class Pid_ctl(object):
    def __init__(self, kP, kI, kD, dt, is_debug=False):
        if is_debug:
            print('PID constructed')
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.dt = dt
        self.is_debug = is_debug
        self.prev_err = 0;
        self.sum_err = 0;

    def control(self, err):
        # Based on the err return the control signal

        self.sum_err += err
        P = self.kP*err
        I = self.kI*self.sum_err*self.dt
        D = self.kD*(err - self.prev_err)/self.dt
        PID = P + I + D

        if self.is_debug:
            print('%s: P = %.4f' %(sys._getframe().f_code.co_name, P))
            print('%s: I = %.4f' %(sys._getframe().f_code.co_name, I))
            print('%s: D = %.4f' %(sys._getframe().f_code.co_name, D))
            print('%s: PID = %.4f' %(sys._getframe().f_code.co_name, PID))

        self.prev_err = err
        return PID

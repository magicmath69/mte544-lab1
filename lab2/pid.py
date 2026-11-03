from rclpy.time import Time
from utilities import Logger

#controller type
P=0 
PD=1
PI=2 
PID=3

class PID_ctrl:
    
    def __init__(self, type_, kp=1.2,kv=0.8,ki=0.2, history_length=3, filename_="errors.csv", integral_limit=None):
        
        #data for the controller
        self.history_length=history_length
        self.history=[]
        self.error_int=0.0
        self.type=type_

        #controller gains
        self.kp=kp
        self.kv=kv
        self.ki=ki

        # anti-windup: max integral action, none = no limit
        self.integral_limit=integral_limit
        
        self.logger=Logger(filename_)
        # use this format when writing toto the file named filename_ or errors.csv:
            # error, error_dot, error_int and time stamp

    
    def update(self, stamped_error, status):
        
        if status == False:
            self.__update(stamped_error)
            return 0.0
        else:
            return self.__update(stamped_error)

        
    def __update(self, stamped_error):
        
        latest_error=stamped_error[0]
        stamp=stamped_error[1]
        stamp_ns=Time.from_msg(stamp).nanoseconds
        repeated_stamp=False

        # integral of error since last sample (trapezoid rule, odometry timestamps)
        if self.history:
            previous_error, previous_stamp=self.history[-1]
            previous_ns=Time.from_msg(previous_stamp).nanoseconds
            dt=(stamp_ns - previous_ns) / 1e9
            if dt > 0:
                self.error_int+=0.5 * (previous_error + latest_error) * dt
            elif dt < 0:
                # sim time reset starts a new error history.
                self.history.clear()
                self.error_int=0.0
            else:
                repeated_stamp=True

        # Part 5: anti-windup clamp. the linear error (a distance) is never negative, so its
        # integral only grows and would keep pushing the robot after it reaches the goal
        if self.integral_limit is not None and self.ki != 0:
            max_error_int=self.integral_limit / abs(self.ki)
            self.error_int=max(-max_error_int, min(max_error_int, self.error_int))

        # accounts for edge case of repeated reading without messing up timestamping
        if repeated_stamp:
            self.history[-1]=stamped_error
        else:
            self.history.append(stamped_error)
        if (len(self.history) > self.history_length):
            self.history.pop(0)

        # Part 5: error derrivative
        # note: the first sample we cant find a derivative for
        error_dot=0.0
        if len(self.history) > 1 and not repeated_stamp:
            first_error, first_stamp=self.history[0]
            elapsed=(stamp_ns - Time.from_msg(first_stamp).nanoseconds) / 1e9
            if elapsed > 0:
                error_dot=(latest_error - first_error) / elapsed

        # Part 4: logging
        self.logger.log_values([latest_error, error_dot, self.error_int, stamp_ns])

        # Part 4: P maps error to a speed
        if self.type == P:
            return self.kp * latest_error
        
        # Part 5: PD, PI and PID
        elif self.type == PD:
            return self.kp * latest_error + self.kv * error_dot
        
        elif self.type == PI:
            return self.kp * latest_error + self.ki * self.error_int
        
        elif self.type == PID:
            return self.kp * latest_error + self.kv * error_dot + self.ki * self.error_int

from rclpy.time import Time
from utilities import Logger

# Controller type
P=0 # poportional
PD=1 # proportional and derivative
PI=2 # proportional and integral
PID=3 # proportional, integral, derivative

class PID_ctrl:
    
    def __init__(self, type_, kp=1.2,kv=0.8,ki=0.2, history_length=3, filename_="errors.csv"):
        
        # Data for the controller
        self.history_length=history_length
        self.history=[]
        self.error_int=0.0
        self.type=type_

        # Controller gains
        self.kp=kp    # proportional gain
        self.kv=kv    # derivative gain
        self.ki=ki    # integral gain
        
        self.logger=Logger(filename_)
        # Remeber that you are writing to the file named filename_ or errors.csv the following:
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

        # Accumulate the area under the error using the actual odometry times.
        # These diagnostics are logged even though P control uses only the error.
        if self.history:
            previous_error, previous_stamp=self.history[-1]
            previous_ns=Time.from_msg(previous_stamp).nanoseconds
            dt=(stamp_ns - previous_ns) / 1e9
            if dt > 0:
                self.error_int+=0.5 * (previous_error + latest_error) * dt
            elif dt < 0:
                # A simulation/odometry time reset starts a new error history.
                self.history.clear()
                self.error_int=0.0
            else:
                repeated_stamp=True

        # A repeated reading replaces the last sample without adding elapsed time.
        if repeated_stamp:
            self.history[-1]=stamped_error
        else:
            self.history.append(stamped_error)
        if (len(self.history) > self.history_length):
            self.history.pop(0)

        # Estimate the slope over the retained history. A first sample or a
        # repeated timestamp has no elapsed interval from which to find a slope.
        error_dot=0.0
        if len(self.history) > 1 and not repeated_stamp:
            first_error, first_stamp=self.history[0]
            elapsed=(stamp_ns - Time.from_msg(first_stamp).nanoseconds) / 1e9
            if elapsed > 0:
                error_dot=(latest_error - first_error) / elapsed

        # Log every sample, including startup, using nanoseconds like robot_pose.csv.
        self.logger.log_values([latest_error, error_dot, self.error_int, stamp_ns])

        # Part 4: proportional control maps the current error to a speed request.
        if self.type == P:
            return self.kp * latest_error
        
        # TODO Part 5: Implement the control law corresponding to each type of controller
        elif self.type == PD:
            raise NotImplementedError("PD control is part of Part 5")
            # return ... # complete
        
        elif self.type == PI:
            raise NotImplementedError("PI control is part of Part 5")
            # return ... # complete
        
        elif self.type == PID:
            raise NotImplementedError("PID control is part of Part 5")
            # return ... # complete

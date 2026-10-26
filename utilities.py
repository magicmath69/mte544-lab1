from math import atan2, pi, sqrt

M_PI=pi

class Logger:

    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"]):

        self.filename = filename

        with open(self.filename, 'w') as file:

            header_str=""

            for header in headers:
                header_str+=header
                header_str+=", "

            header_str+="\n"

            file.write(header_str)


    def log_values(self, values_list):

        with open(self.filename, 'a') as file:

            vals_str=""

            for value in values_list:
                vals_str+=f"{value}, "

            vals_str+="\n"

            file.write(vals_str)


    def save_log(self):
        # Each log_values call closes its file, so the data is already saved.
        pass

class FileReader:
    def __init__(self, filename):

        self.filename = filename


    def read_file(self):

        read_headers=False

        table=[]
        headers=[]
        with open(self.filename, 'r') as file:

            if not read_headers:
                for line in file:
                    values=line.strip().split(',')

                    for val in values:
                        if val=='':
                            break
                        headers.append(val.strip())

                    read_headers=True
                    break

            # The header loop already consumed the header; keep the first data row.

            # Read each line and extract values
            for line in file:
                values = line.strip().split(',')

                row=[]

                for val in values:
                    if val=='':
                        break
                    row.append(float(val.strip()))

                table.append(row)

        return headers, table



# Part 3: Convert the odometry quaternion to planar heading.
def euler_from_quaternion(quat):
    """
    Return yaw in radians from a quaternion with w in last place.
    quat = [x, y, z, w]
    """

    # Only yaw is needed for motion on the floor.
    x, y, z, w = quat
    yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return yaw


# Part 4: Calculate the distance to the goal.
def calculate_linear_error(current_pose, goal_pose):

    # current_pose = [x, y, theta, stamp], goal_pose = [x, y].
    dx=goal_pose[0] - current_pose[0]
    dy=goal_pose[1] - current_pose[1]
    error_linear=sqrt(dx**2 + dy**2)

    return error_linear

# Part 4: Calculate the shortest turn toward the goal.
def calculate_angular_error(current_pose, goal_pose):

    dx=goal_pose[0] - current_pose[0]
    dy=goal_pose[1] - current_pose[1]
    # At the goal there is no target direction, so request no rotation.
    if dx == 0 and dy == 0:
        return 0.0

    desired_heading=atan2(dy, dx)
    error_angular=desired_heading - current_pose[2]

    # Wrap to [-pi, pi] so the robot takes the shorter turning direction.
    error_angular=(error_angular + M_PI) % (2*M_PI) - M_PI

    return error_angular

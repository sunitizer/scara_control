import numpy as np
import matplotlib.pyplot as plt
import math
import time

def main():
    start_time = time.time()
    #testing
    pos1 = [250, 50]
    pos2 = [-5, 200]
    speed = 20 #mm/s
    path_calculator = ScaraPathCalculator(arm_len1=179.9,
                                          arm_len2=150,
                                          angle_res1=2*math.pi/200/3.70588235/8,
                                          angle_res2=2*math.pi/400/2/8)
    
    coef1 = path_calculator.line_compute_times(pos1, pos2, speed)
    print("Time elapsed: {}".format(time.time()-start_time))
    print(coef1)
    
class ScaraPathCalculator(object):
    """ Class for managing compution of scara arm coefficients
    """
    def __init__(self, arm_len1, arm_len2, angle_res1, angle_res2):
        # length of each arm
        self.ARM_LEN1 = arm_len1
        self.ARM_LEN2 = arm_len2

        # the angle each motor rotates per step
        self.ANGLE_RES1 = angle_res1
        self.ANGLE_RES2 = angle_res2

    def line_compute_times(self, pos_start, pos_end, speed, time_res=0.001):
        """
        Computes the parameters needed for running end effector in a straight line
        pos_start and pos_end are starting and end positions, both are in the format of [x, y]
        speed is mm/s linear speed
        time_res is the time resolution in seconds
        """
        # find discretized positions of the SCARA arm
        disc_angle_start = self.find_discrete_angle(pos_start)
        disc_angle_end = self.find_discrete_angle(pos_end)

        disc_pos_start = self.find_pos(disc_angle_start)
        disc_pos_end = self.find_pos(disc_angle_end)

        # slope m and y-intercept b computed
        m = (disc_pos_end[1] - disc_pos_start[1]) / (disc_pos_end[0] - disc_pos_start[0])
        b = disc_pos_start[1] - m * disc_pos_start[0]

        # find time it takes to complete linear run with given speed
        disc_dist = self.distance_two_points(disc_pos_start, disc_pos_end)
        total_time = disc_dist / speed

        # define time array of 0 to 1 in time_res increments
        time_array = np.linspace(0, 1, 1/time_res+1)

        # solve for angles array for both motors given time_array
        # Index time_array backwards with [::-1] to achieve same thing as 1-time
        x_t = time_array[::-1]*disc_pos_start[0] + time_array*disc_pos_end[0]
        # Automatic vectorized calculation of y_t, since x_t is an np.ndarray
        y_t = m*x_t + b
        # Use numpy versions of trig functions so that everything is vectorized
        theta2_t = np.arccos((x_t**2 + y_t**2 - self.ARM_LEN1**2 - self.ARM_LEN2**2) / (2*self.ARM_LEN1*self.ARM_LEN2)) 
        theta1_t = (np.arctan2(y_t, x_t) + np.arctan2((self.ARM_LEN2*np.sin(theta2_t)), (self.ARM_LEN1 + self.ARM_LEN2*np.cos(theta2_t))))
        
        # convert time to right time
        time_array *= total_time

        # Initialize 1D lists, can append lists each iteration through the loop if needed
        step_arrays = []
        direction = []

        # Uses theta1_t, self.ANGLE_RES1 first time through loop
        # Uses theta2_t, self.ANGLE_RES2 second time through loop
        angles_arrays = [theta1_t, theta2_t]
        resolution_array = [self.ANGLE_RES1, self.ANGLE_RES2]
        for angles, res in zip(angles_arrays, resolution_array):
            # will check if the motor will run in opposite direction, if so will adjust graph and find point of dir change
            maxmin_index = self.check_for_max_min(angles)
            if maxmin_index is not None:
                turning_angle = angles[maxmin_index]
                split_angles_arrays = [angles[:maxmin_index + 1], angles[maxmin_index:]-turning_angle]

                # convert angles array to steps array
                split_step_arrays = [self.angle_to_steps(arr, res) for arr in split_angles_arrays]
                # print(split_step_arrays[0][1])
                turning_step = split_step_arrays[0][-1]
                split_step_arrays[1] += turning_step

                # Use hstack to concatenate numpy ndarrays, equivalent of adding lists
                step_arrays.append(np.hstack([split_step_arrays[0], split_step_arrays[1][1:]]))

                direction.append([self.motor_dir(angles[0], angles[maxmin_index]), maxmin_index])
            else:
                step_arrays.append(self.angle_to_steps(angles, res))
                direction.append([self.motor_dir(angles[0], angles[-1]), None])

        # fits polynomial to the graphs
        quad_coef = [np.polyfit(step_arrays[0], time_array, 3), np.polyfit(step_arrays[1], time_array, 3)]
        self.plot_fit(step_arrays[0], time_array, quad_coef[0])
        self.plot_fit(step_arrays[1], time_array, quad_coef[1])

        num_steps = [round(arr[-1]) for arr in step_arrays]

        return [quad_coef[0], quad_coef[1], num_steps, direction]
        #return [quad_coef[0][0], quad_coef[0][1], quad_coef[0][2], quad_coef[1][0], quad_coef[1][1], quad_coef[1][2], num_steps[0], num_steps[1], direction]

    def find_discrete_angle(self, pos_start):
        """find discretized angle of given start_position based on ANGLE_RES of both motors"""
        actual_angle = self.find_angles(pos_start)
        float_steps = [round(actual_angle[0]/self.ANGLE_RES1), round(actual_angle[1]/self.ANGLE_RES2)]
        discrete_angle = [float_steps[0]*self.ANGLE_RES1, float_steps[1]*self.ANGLE_RES2]

        return discrete_angle

    def find_angles(self, pos):
        """find angle of motors 1 and 2 in radian given position using inverse kinematics"""
        exact_angle = [0, 0]

        exact_angle[1] = math.acos((pos[0]**2 + pos[1]**2 - self.ARM_LEN1**2 - self.ARM_LEN2**2) / (2*self.ARM_LEN1*self.ARM_LEN2))
        exact_angle[0] = (math.atan2(pos[1], pos[0])
                        + math.atan2(self.ARM_LEN2*math.sin(exact_angle[1]), self.ARM_LEN1+self.ARM_LEN2*math.cos(exact_angle[1])))
        return exact_angle

    def find_pos(self, angle):
        """forward kinematics to find position of end-effector given angle"""
        pos = [self.ARM_LEN1 * math.cos(angle[0]) + self.ARM_LEN2 * math.cos(angle[0] - angle[1]),
               self.ARM_LEN1 * math.sin(angle[0]) + self.ARM_LEN2 * math.sin(angle[0] - angle[1])]
        return pos


    def angle_to_steps(self, angle_array, resolution):
        """takes the initial angle and final angle and resolution per step and returns the steps in a np.ndarray"""
        initial = angle_array[0]
        return np.abs(angle_array-initial)/resolution
    

    def distance_two_points(self, point1, point2):
        """Find the distance between two points"""
        distance = math.sqrt((point2[0]-point1[0])**2 + (point2[1]-point1[1])**2)
        return distance


    def plot_fit(self, x_array, y_array, coef):
        """plots the x and y array and also the polynomial of 3rd order using the coefficients"""
        size = int(x_array[-1])
        x1 = np.arange(size)
        y = coef[0]*x1**3+coef[1]*x1**2+coef[2]*x1**1+coef[3]
        
        # print(x1)
        plt.plot(x_array, y_array, 'bo', x1, y, 'r--')
        plt.show()


    def check_for_max_min(self, angle_array):
        """Checks for global max or min and returns the array index at that point,
        or None if angle_array is monotonic
        """
        argmax = np.argmax(angle_array)
        argmin = np.argmin(angle_array)
        array_len = len(angle_array)
        if argmax != 0 and argmax != array_len-1:
            return argmax
        elif argmin != 0 and argmin != array_len-1:
            return argmin
        else:
            return None


    def motor_dir(self, angle_start, angle_end):
        """ returns True for positive direction theta and False for negative direction"""
        return angle_end > angle_start


if __name__ == "__main__":
    main()

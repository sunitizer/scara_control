import numpy as np
import matplotlib.pyplot as plt
import math
import time

# length of each arm
ARM_LEN1 = 179.9
ARM_LEN2 = 150

# the angle each motor rotates per step
ANGLE_RES1 = 2*math.pi/200/3.70588235/8
ANGLE_RES2 = 2*math.pi/400/2/8


def line_compute_times(pos_start, pos_end, speed):
    """
    Computes the parameters needed for running end effector in a straight line
    pos_start and pos_end are starting and end positions, both are in the format of [x, y]
    speed is mm/s linear speed
    """
    # find discretized positions of the SCARA arm
    disc_angle_start = find_discrete_angle(find_angles(pos_start))
    disc_angle_end = find_discrete_angle(find_angles(pos_end))

    disc_pos_start = find_pos(disc_angle_start)
    disc_pos_end = find_pos(disc_angle_end)

    # slope m and y-intercept b computed
    m = (disc_pos_end[1] - disc_pos_start[1]) / (disc_pos_end[0] - disc_pos_start[0])
    b = disc_pos_start[1] - m * disc_pos_start[0]

    # find time it takes to complete linear run with given speed
    disc_dist = distance_two_points(disc_pos_start, disc_pos_end)
    total_time = disc_dist / speed

    # define time array of 0 to 1 in 0.001 increments
    time_array = np.linspace(0, 1, 1001)

    # solve for angles array for both motors given time_array
    angles_arrays = [[], []]
    for time in time_array:
        x_t = (1-time)*disc_pos_start[0] + time*disc_pos_end[0]
        y_t = m*x_t + b
        theta2_t = math.acos((x_t**2+y_t**2-ARM_LEN1**2-ARM_LEN2**2)/(2*ARM_LEN1*ARM_LEN2))
        theta1_t = (math.atan2(y_t, x_t)
                    + math.atan2((ARM_LEN2*math.sin(theta2_t)), (ARM_LEN1+ARM_LEN2*math.cos(theta2_t))))
        angles_arrays[0].append(theta1_t)
        angles_arrays[1].append(theta2_t)

    # convert time to right time
    time_array = [time_mag * total_time for time_mag in time_array]

    step_arrays = [[], []]
    direction = [[], []]

    # will check if the motor will run in opposite direction, if so will adjust graph and find point of dir change
    maxmin_index = [check_for_max_min(angles_arrays[0]), check_for_max_min(angles_arrays[1])]
    if maxmin_index[0] is not None:

        split_step_arrays = [[], []]
        split_angles_arrays = [angles_arrays[0][:maxmin_index[0] + 1], angles_arrays[0][maxmin_index[0]:]]
        split_angles_arrays[1] = [shift_angle - split_angles_arrays[1][0] for shift_angle in
                                  split_angles_arrays[1]]

        # convert angles array to steps array
        split_step_arrays[0] = angle_to_steps(split_angles_arrays[0], ANGLE_RES1)
        split_step_arrays[1] = angle_to_steps(split_angles_arrays[1], ANGLE_RES1)

        # print(split_step_arrays[0][1])
        split_step_arrays[1] = [split_step_arrays[0][-1] + shift_step for shift_step in
                                   split_step_arrays[1]]

        step_arrays[0] = split_step_arrays[0] + split_step_arrays[1][1:]

        direction[0].append(motor_dir(angles_arrays[0][0], angles_arrays[0][maxmin_index[0]]))
        direction[0].append(maxmin_index[0])

    else:
        step_arrays[0] = angle_to_steps(angles_arrays[0], ANGLE_RES1)
        direction[0].append(motor_dir(angles_arrays[0][0], angles_arrays[0][-1]))
        direction[0].append(None)

    if maxmin_index[1] is not None:
        split_step_arrays = [[], []]
        split_angles_arrays = [angles_arrays[1][:maxmin_index[1] + 1], angles_arrays[1][maxmin_index[1]:]]
        split_angles_arrays[1] = [shift_angle - split_angles_arrays[1][0] for shift_angle in
                                  split_angles_arrays[1]]

        # convert angles array to steps array
        split_step_arrays[0] = angle_to_steps(split_angles_arrays[0], ANGLE_RES2)
        split_step_arrays[1] = angle_to_steps(split_angles_arrays[1], ANGLE_RES2)

        # print(split_step_arrays[0][1])
        split_step_arrays[1] = [split_step_arrays[0][-1] + shift_step for shift_step in
                                split_step_arrays[1]]

        step_arrays[1] = split_step_arrays[0] + split_step_arrays[1][1:]

        direction[1].append(motor_dir(angles_arrays[1][0], angles_arrays[1][maxmin_index[1]]))
        direction[1].append(maxmin_index[1])
    else:
        step_arrays[1] = angle_to_steps(angles_arrays[1], ANGLE_RES2)
        direction[1].append(motor_dir(angles_arrays[1][0], angles_arrays[1][-1]))
        direction[1].append(None)

    # fits polynomial to the graphs
    quad_coef = [np.polyfit(step_arrays[0], time_array, 3), np.polyfit(step_arrays[1], time_array, 3)]
    plot_fit(step_arrays[0], time_array, quad_coef[0])
    plot_fit(step_arrays[1], time_array, quad_coef[1])

    num_steps = [round(step_arrays[0][-1]), round(step_arrays[1][-1])]

    return_array = [quad_coef[0], quad_coef[1], num_steps, direction]
    #return_array = [quad_coef[0][0], quad_coef[0][1], quad_coef[0][2], quad_coef[1][0], quad_coef[1][1], quad_coef[1][2], num_steps[0], num_steps[1], direction]
    return return_array


def find_angles(pos):
    """find angle of motors 1 and 2 in radian given position using inverse kinematics"""
    exact_angle = [0, 0]

    exact_angle[1] = math.acos((pos[0]**2 + pos[1]**2 - ARM_LEN1**2 - ARM_LEN2**2) / (2*ARM_LEN1*ARM_LEN2))
    exact_angle[0] = (math.atan2(pos[1], pos[0])
                      + math.atan2(ARM_LEN2*math.sin(exact_angle[1]), ARM_LEN1+ARM_LEN2*math.cos(exact_angle[1])))
    return exact_angle


def find_discrete_angle(actual_angle):
    """find discretized angle of given angle based on ANGLE_RES of both motors"""
    float_steps = [round(actual_angle[0]/ANGLE_RES1), round(actual_angle[1]/ANGLE_RES2)]
    discrete_angle = [float_steps[0]*ANGLE_RES1, float_steps[1]*ANGLE_RES2]

    return discrete_angle


def find_pos(angle):
    """forward kinematics to find position of end-effector given angle"""
    pos = [ARM_LEN1 * math.cos(angle[0]) + ARM_LEN2 * math.cos(angle[0] - angle[1]),
           ARM_LEN1 * math.sin(angle[0]) + ARM_LEN2 * math.sin(angle[0] - angle[1])]

    return pos


def angle_to_steps(angle_array, resolution):
    """takes the initial angle and final angle and resolution per step and returns the steps in a list"""
    initial = angle_array[0]
    steps = []
    for angle in angle_array:
        steps.append(abs(angle - initial)/resolution)

    return steps


def distance_two_points(point1, point2):
    """Find the distance between two points"""
    distance = math.sqrt((point2[0]-point1[0])**2 + (point2[1]-point1[1])**2)
    return distance


def plot_fit(x_array, y_array, coef):
    """plots the x and y array and also the polynomial of 3rd order using the coefficients"""
    size = int(x_array[-1])
    x1 = range(0, size)
    y = []
    for x in x1:
        y.append(coef[0]*x**3+coef[1]*x**2+coef[2]*x**1+coef[3])

    # print(x1)
    plt.plot(x_array, y_array, 'bo', x1, y, 'r--')
    plt.show()


def check_for_max_min(angle_array):
    """checks for global max or min and returns the array index at that point"""
    maxmin_index = None
    for index in range(1, len(angle_array)-1):
        if angle_array[index] >= angle_array[index-1] and angle_array[index] >= angle_array[index+1]:
            maxmin_index = index
        elif angle_array[index] <= angle_array[index-1] and angle_array[index] <= angle_array[index+1]:
            maxmin_index = index

    # print(maxmin_index)
    return maxmin_index


def motor_dir(angle_start, angle_end):
    """ returns True for positive direction theta and False for negative direction"""
    if angle_end > angle_start:
        direction = True
    else:
        direction = False
    return direction

start_time = time.time()
#testing
pos1 = [250, 50]
pos2 = [-5, 200]
coef1 = line_compute_times(pos1, pos2, 20)
print("Time elapsed: {}".format(time.time()-start_time))
print(coef1)

#!/usr/bin/env python3

# return the MSE between rtr control and teach pendant controlled data.
import argparse
from pathlib import PurePath
import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import mean_squared_error

# from rtr_control_ros.log_plots import *
from rtr_control_ros.log_plot_utils import *
from rtr_control_ros.robot_logs import RobotLogs

RAD2DEG = 180 / np.pi

# isolate interesting parts
# get times when velocity nonzero
def get_end_times(data, threshold=0.05):
    # get starting time, end time
    start_time = []
    end_time = []
    for i in range(len(data[0])):
        vals = [data[j][i] for j in range(len(data))]
        start_index = next(n for n,v in enumerate(vals) if abs(v) > threshold)
        end_index = next(n for n,v in enumerate(vals[::-1]) if abs(v) > threshold)

        start_time.append(start_index)
        end_time.append(len(data)-end_index-1) # because order reversed

    return (start_time, end_time)

def plot_data(time, data, mse):
    fig = plt.figure()

    # different subplot joint charts
    for i in range(len(data[0])):
        ax = plt.subplot(len(data[0]),1,i+1)
        # print('new joint')

        # different lines on each subplot
        for j in range(len(data)):
            # print('new line')
            # print("time", len(time[j][i]))
            # print("pos", len(data[j][i]))
            plt.plot(time[j][i], data[j][i])

        plt.title("Joint: {}, MSE: {:.3g}".format(i, mse[i]))
    return

def get_mse(data):
    # returns mse of each joint rtr control compared to teach pendant
    mse = []
    for i in range(len(data[0])):
        first = np.array(data[0][i])
        second = np.array(data[1][i])
        # zero-pad arrays to be same length
        if len(first)<len(second):
            first.resize(len(second))
        else:
            second.resize(len(first))

        mse.append(mean_squared_error(first, second))
    return mse

def get_data(fname, fname2):
    files = [fname, fname2]

    pos,vel,time = [],[],[]

    for f in files:
        fpath = PurePath(f)

        robot_name = fpath.stem
        rlogs = RobotLogs(robot_name)
        rlogs.extract_data(f)

        # assumes t interval is relatively constant
        t, p_c, p_r = rlogs.get_pos_data()
        _, v_c, v_r = rlogs.get_vel_data()
        start, end = get_end_times(v_r, 0.1)

        # print(robot_name)
        # print(t[0])
        # print(np.shape(p_r))
        p_r = np.transpose(p_r)
        # p_c = np.transpose(p_c)
        # print(np.shape(p_r))

        p_r_trim, v_r_trim = [], []
        t_trim = []

        # extract only interesting data
        for i in range(len(p_r)):
            # print(start[i],end[i])
            # print(end[i]-start[i])
            p_r_trim.append(p_r[i][start[i]:end[i]])
            v_r_trim.append(v_r[i][start[i]:end[i]])
            t_trim.append([j-t[start[i]] for j in t[start[i]:end[i]]]) # normalize times

        pos.append(p_r_trim)
        vel.append(v_r_trim)
        time.append(t_trim)

    return pos, vel, time

def main():
    parser = argparse.ArgumentParser(description="Basic plotting script for control data")
    parser.add_argument("-f",
                        "--file",
                        help="File path to binary file. Required argument.",
                        type=str,
                        required=True)
    parser.add_argument("-f2",
                        "--file2",
                        help="File path to second binary file. Optional argument",
                        type=str,
                        required=True)

    args = parser.parse_args()
    fname = args.file
    fname2 = args.file2


    pos,vel,time = get_data(fname, fname2)

    # get MSE of pos
    mse = get_mse(pos)

    # plot pos
    plot_data(time, pos, mse)
    # plot_data(t_trim, vel)

    # print("Plotting all trajectories")
    # construct_trajectory_plots_isolated(rlogs, type='pvaj', in_degrees=False)
    # plt.show()
    # construct_trajectory_plots_isolated(rlogs2, type='pvaj', in_degrees=False)
    #
    plt.show()

if __name__ == "__main__":
    main()

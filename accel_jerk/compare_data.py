#!/usr/bin/env python3

# return the MSE between rtr control and teach pendant controlled data.
import argparse
from pathlib import PurePath
import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import mean_squared_error

# from rtr_control_ros.log_plots import *
# from rtr_control_ros.log_plot_utils import *
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
        start_index = next(n for n, v in enumerate(vals) if abs(v) > threshold)
        end_index = next(n for n, v in enumerate(vals[::-1]) if abs(v) > threshold)

        start_time.append(start_index)
        end_time.append(len(data) - end_index - 1)  # because order reversed

    return (start_time, end_time)


def plot_data(time, data, mse):
    fig = plt.figure()

    # different subplot joint charts
    for i in range(len(data[0])):
        ax = plt.subplot(len(data[0]), 1, i + 1)

        # different lines on each subplot
        for j in range(len(data)):
            # print("time", len(time[j][i]))
            # print("pos", len(data[j][i]))
            plt.plot(time[j][i], data[j][i])

        plt.title("Joint: {}, MSE: {:.3g}".format(i, mse[i]))
    return


def get_mse(data, pred):
    # returns mse of each joint rtr control compared to teach pendant
    mse = []
    for i in range(len(data)):
        j_data = np.array(data[i])
        j_pred = np.array(pred[i])
        # zero-pad arrays to be same length
        if len(j_data) < len(j_pred):
            j_data.resize(len(j_pred))
        else:
            j_pred.resize(len(j_data))

        mse.append(mean_squared_error(j_data, j_pred))
    return mse


def get_data(fname):
    fpath = PurePath(fname)

    robot_name = fpath.stem
    rlogs = RobotLogs(robot_name)
    rlogs.extract_data(fname)

    # assumes t interval is relatively constant
    t, p_c, p_r = rlogs.get_pos_data()
    _, v_c, v_r = rlogs.get_vel_data()
    start, end = get_end_times(v_r, 0.1)

    p_r = np.transpose(p_r)
    # p_c = np.transpose(p_c)

    p_r_trim, v_r_trim = [], []
    t_trim = []

    # extract only interesting data
    for i in range(len(p_r)):
        # print(start[i],end[i])
        # print(end[i]-start[i])
        p_r_trim.append(p_r[i][start[i]:end[i]])
        v_r_trim.append(v_r[i][start[i]:end[i]])
        t_trim.append([j - t[start[i]] for j in t[start[i]:end[i]]])  # normalize times

    # pos.append(p_r_trim)
    # vel.append(v_r_trim)
    # time.append(t_trim)

    # return pos, vel, time
    return p_r_trim, v_r_trim, t_trim


def main():
    parser = argparse.ArgumentParser(description="Basic plotting script for control data")
    parser.add_argument(
        "-f",
        "--file",
        help="File path to binary file. Required argument.",
        type=str,
        required=True)
    parser.add_argument(
        "-f2",
        "--file2",
        help="File path to second binary file. Optional argument",
        type=str,
        required=True)

    args = parser.parse_args()
    fname = args.file
    fname2 = args.file2

    pos, vel, time = [], [], []
    pos = [get_data(fname)[0], get_data(fname2)[0]]
    time = [get_data(fname)[2], get_data(fname2)[2]]

    # get MSE of pos
    mse = get_mse(pos[0], pos[1])

    # plot pos
    plot_data(time, pos, mse)
    # plot_data(t_trim, vel)

    plt.show()


if __name__ == "__main__":
    main()

from rtr_control_ros.log_plot_utils import *
"""Control Plot Constructors"""

RAD2DEG = 180 / np.pi


def step_through_trajectories(robot_logs, ws_logs=None, indices=None, type='pvaj', in_degrees=True):
    """Plots all/subset of commanded trajectories one at a time
    
    Plots a single trajectory and then waits for user input before plotting the next trajectory.

    Args:
        robots_logs (RobotLogs): parsed robot logs
        ws_logs (WSLogs, optional): parsed wireshark logs. Default is None.
        indices (arr, optional): array of trajectory indices. Default is None.
        type (str, optional): select which trajectory to plot. Default is all.
            'p' - position
            'v' - velocity
            'a' - acceleration
            'j' - jerk
        in_degrees (bool, optional): True to plot in degrees. Default is True.

    """
    num_traj = robot_logs.get_num_trajectories()
    traj_iters = range(num_traj)
    if indices:
        traj_iters = indices

    for i in traj_iters:
        if i >= num_traj:
            raise IndexError("Index ({}) exceeds number of trajectories ({})".format(i, num_traj))

        construct_trajectory_plots(robot_logs, ws_logs, i, type=type, in_degrees=in_degrees)
        plt.show(block=False)
        ret = input(
            "Showing trajectory {} of {}. Press enter for next plot (or q to quit)\n".format(
                i + 1, num_traj))
        plt.close('all')

        if ret == 'q':
            break


def construct_trajectory_plots(robot_logs,
                               ws_logs=None,
                               i=None,
                               adjust_delay=False,
                               highlight_indices=None,
                               type='pvaj',
                               in_degrees=True):
    """Constructs plot for an arbitrary trajectory

    Must call pyplot.show() after calling this function to display the plot

    Args:
        robots_logs (RobotLogs): parsed robot logs
        ws_logs (WSLogs, optional): parsed wireshark logs. Default is None
        i (int, optional): the ith trajectory. Default is None (plot everything)
        adjust_delay (bool, optional): plot data adjusted for delay between 
            controller and robot. Default is False.
        highlight_indices (array, optional): array of indexes. Highlights 
           points of interested at ith position along x-axis. Default is None.
        type (str, optional): select which trajectory to plot. Default is all.
            'p' - position
            'v' - velocity
            'a' - acceleration
            'j' - jerk
        in_degrees (bool, optional): True to plot in degrees. Default is True.

    """
    t, p_c, p_r = robot_logs.get_pos_data(i, adjust_delay)
    _, v_c, v_r = robot_logs.get_vel_data(i, adjust_delay)
    _, a_c, a_r = robot_logs.get_accel_data(i, adjust_delay)
    _, j_c, j_r = robot_logs.get_jerk_data(i, adjust_delay)

    unit = 'rad'
    if in_degrees:
        unit = 'deg'
        p_c *= RAD2DEG
        p_r *= RAD2DEG
        v_c *= RAD2DEG
        v_r *= RAD2DEG
        a_c *= RAD2DEG
        a_r *= RAD2DEG
        j_c *= RAD2DEG
        j_r *= RAD2DEG

    shade_regions = None
    if ws_logs:
        region_data = ws_logs.get_network_jitter_data(i)
        if region_data:
            label = 'jitter > {}s'.format(ws_logs.jitter_cutoff)
            shade_regions = PlotAttr(region_data, label, 'b')

    highlight_data = None
    if highlight_indices:
        highlight_data = PlotAttr(highlight_indices, 'highlight', 'b*')

    xlabel = 'time (s)'

    if 'p' in type:
        pos_data = YData()
        pos_data.append(p_c, 'command', 'g')
        pos_data.append(p_r, 'robot', 'r:')
        basic_joint_plot(t,
                         pos_data,
                         shade_regions=shade_regions,
                         highlight_data=highlight_data,
                         xlabel=xlabel,
                         ylabel=unit,
                         title='{} - Position Readings'.format(robot_logs.name))

    if 'v' in type:
        vel_data = YData()
        vel_data.append(v_c, 'command', 'g')
        vel_data.append(v_r, 'robot', 'r:')
        basic_joint_plot(t,
                         vel_data,
                         shade_regions=shade_regions,
                         highlight_data=highlight_data,
                         xlabel=xlabel,
                         ylabel='{}/s'.format(unit),
                         title='{} - Velocity Readings'.format(robot_logs.name))

    if 'a' in type:
        accel_data = YData()
        accel_data.append(a_c, 'command', 'g')
        accel_data.append(a_r, 'robot', 'r:')
        basic_joint_plot(t,
                         accel_data,
                         shade_regions=shade_regions,
                         highlight_data=highlight_data,
                         xlabel=xlabel,
                         ylabel='{}/$s\u00B2$'.format(unit),
                         title='{} - Acceleration Readings'.format(robot_logs.name))

    if 'j' in type:
        jerk_data = YData()
        jerk_data.append(j_c, 'command', 'g')
        jerk_data.append(j_r, 'robot', 'r:')
        basic_joint_plot(t,
                         jerk_data,
                         shade_regions=shade_regions,
                         highlight_data=highlight_data,
                         xlabel=xlabel,
                         ylabel='{}/$s\u00B3$'.format(unit),
                         title='{} - Jerk Readings'.format(robot_logs.name))


def construct_tcp_plots(robot_logs, i=None):
    xlabel = 'time (s)'

    # position
    t, cmd_pos, cmd_rot, robot_pos, robot_rot = robot_logs.get_tcp_pos(i)
    pos_data = YData()
    pos_data.append(cmd_pos, 'command', 'g')
    pos_data.append(robot_pos, 'robot', 'r:')

    rot_data = YData()
    rot_data.append(cmd_rot, 'command', 'g')
    rot_data.append(robot_rot, 'robot', 'r:')
    basic_pose_plot(t,
                    pos_data,
                    rot_data,
                    xlabel=xlabel,
                    title='{} - TCP Pose'.format(robot_logs.name))

    # velocity
    t, cmd_vel, cmd_angular_vel, robot_vel, robot_angular_vel = robot_logs.get_tcp_vel(i)

    vel_data = YData()
    vel_data.append(cmd_vel, 'command', 'g')
    vel_data.append(robot_vel, 'robot', 'r:')

    ang_vel_data = YData()
    ang_vel_data.append(cmd_angular_vel, 'command', 'g')
    ang_vel_data.append(robot_angular_vel, 'robot', 'r:')
    basic_pose_plot(t,
                    vel_data,
                    ang_vel_data,
                    xlabel=xlabel,
                    ylabel=['m/s', 'rad/s'],
                    title='{} - TCP Velocity'.format(robot_logs.name))


def construct_error_plots(robot_logs,
                          i,
                          ws_logs=None,
                          adjust_delay=False,
                          type='pvaj',
                          in_degrees=True):
    """Plots error between command and robot trajectory

    Must call pyplot.show() after calling this function to display the plot

    Args:
        robots_logs (RobotLogs): parsed robot logs
        ws_logs (WSLogs, optional): parsed wireshark logs
        i (int): the ith trajectory. None = plot everything
        adjust_delay (bool, optional): plot data adjusted for delay between 
            controller and robot
        type (str, optional): select which trajectory to plot
            'p' - position
            'v' - velocity
            'a' - acceleration
            'j' - jerk
        in_degrees (bool, optional): True to plot in degrees. Default is True.

    """
    t, p_c, p_r = robot_logs.get_pos_data(i, adjust_delay)
    _, v_c, v_r = robot_logs.get_vel_data(i, adjust_delay)
    _, a_c, a_r = robot_logs.get_accel_data(i, adjust_delay)
    _, j_c, j_r = robot_logs.get_jerk_data(i, adjust_delay)

    p_error = p_c - p_r
    v_error = v_c - v_r
    a_error = a_c - a_r
    j_error = j_c - j_r

    unit = 'rad'
    if in_degrees:
        p_error *= RAD2DEG
        v_error *= RAD2DEG
        a_error *= RAD2DEG
        j_error *= RAD2DEG
        unit = 'deg'

    shade_regions = None
    if ws_logs:
        region_data = ws_logs.get_network_jitter_data(i)
        if region_data:
            label = 'jitter > {}s'.format(ws_logs.jitter_cutoff)
            shade_regions = PlotAttr(region_data, label, 'b')

    xlabel = 'time (s)'

    if 'p' in type:
        perror_data = YData()
        perror_data.append(p_error, 'error', 'r')
        basic_joint_plot(t,
                         perror_data,
                         shade_regions=shade_regions,
                         xlabel=xlabel,
                         ylabel=unit,
                         title='{} - Position Error'.format(robot_logs.name))

    if 'v' in type:
        verror_data = YData()
        verror_data.append(v_error, 'error', 'r')
        basic_joint_plot(t,
                         verror_data,
                         shade_regions=shade_regions,
                         xlabel=xlabel,
                         ylabel='{}/s'.format(unit),
                         title='{} - Velocity Error'.format(robot_logs.name))

    if 'a' in type:
        aerror_data = YData()
        aerror_data.append(a_error, 'error', 'r')
        basic_joint_plot(t,
                         aerror_data,
                         shade_regions=shade_regions,
                         xlabel=xlabel,
                         ylabel='{}/$s^2$'.format(unit),
                         title='{} - Acceleration Error'.format(robot_logs.name))

    if 'j' in type:
        jerror_data = YData()
        jerror_data.append(j_error, 'error', 'r')
        basic_joint_plot(t,
                         jerror_data,
                         shade_regions=shade_regions,
                         xlabel=xlabel,
                         ylabel='{}/$s^3$'.format(unit),
                         title='{} - Jerk Error'.format(robot_logs.name))

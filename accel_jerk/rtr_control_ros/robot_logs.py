import pandas as pd
import msgpack
import numpy as np
from scipy.spatial.transform import Rotation, RotationSpline

from rtr_control_ros.log_parse import read_control_logs


class RobotLogs(object):
    """The basic container for storing controller log data

    RobotLogs is used to parse robot logs dumped by the RTR SplineController
    
    Attributes:
        name (str): user-defined ID for log data. Used for labeling plot.

    """

    CMD_CONFIG = "cmd_config"
    CMD_VEL = "cmd_velocity"
    ROBOT_CONFIG = "robot_config"
    ROBOT_VEL = "robot_velocity"
    CTRL_TIME = "cmd_time"
    SYS_TIME = "time"
    CMD_TCP = "cmd_tcp"
    ROBOT_TCP = "robot_tcp"

    def __init__(self, name):
        """Constructor

        Args:
            name (str): robot name

        """
        self.name = name
        self._raw_data = None
        self._data_start = None
        self._data_end = None
        self._column_ref = None
        self._num_dof = None

        # Filtered
        self._update_rate = None
        self._traj_index_pairs = None
        self._adjusted_delay_index = None
        self._delay_hist = None
        self._delay_estimated = False

    def extract_data(self, fname, cutoff=(0, 1), update_rate=None):
        """Parses binary file and extracts raw data

        Args:
            fname (str): file path to binary file
            cutoff (tuple, optional): (start, end) percent of data to extract. 
                Example: (0.25, 0.75) reads all data between the first 25% and 
                first 75% of the data. Default is (0,1)
        
        """

        self._raw_data = read_control_logs(fname, cutoff)

        # Find degrees of freedom + start index
        for i, val in enumerate(self._raw_data[self.CMD_CONFIG]):
            if len(val) > 0:
                self._num_dof = len(val)
                self._data_start = i
                break

        # post-process any empty configs
        config_keys = [
            self.CMD_CONFIG, self.CMD_VEL, self.ROBOT_CONFIG, self.ROBOT_VEL, self.CMD_TCP,
            self.ROBOT_TCP
        ]
        nan_config = [float("Nan")] * self._num_dof
        for key in config_keys:
            col = self._raw_data[key].apply(lambda x: nan_config if len(x) == 0 else x)
            self._raw_data[key] = col

        # Set end index based on first instance of null timestamp
        null_timestamp = msgpack.ext.Timestamp(0)
        self._data_end = np.where(self._raw_data[self.SYS_TIME] == null_timestamp)[0][0]

        # Mapping of column names to column number
        self._column_ref = {k: self._raw_data.columns.get_loc(k) for k in self._raw_data.columns}

    def filter_data(self, update_rate=None):
        """Filters data and extracts trajectory-specific information

        Args:
            update_rate (float, optional): update rate of the robot. If none, 
                then function will attempt to automatically infer the update 
                rate from data. Extracting the update rate automatically can 
                lead to spurious results if data is sparse. Default is None.

        """
        cmd_time = self._raw_data[self.CTRL_TIME]

        # Find update rate
        if update_rate is None:
            for val in np.unique(cmd_time):
                if val > 0:
                    self._update_rate = val
                    break
        else:
            self._update_rate = update_rate

        # Throw if no update rate
        if self._update_rate is None:
            raise ValueError("update rate is None")

        # Extract start/end of all trajectories
        start_instances = np.where(cmd_time == self._update_rate)[0]
        self._traj_index_pairs = []
        for i in range(1, len(start_instances)):
            start_index = start_instances[i - 1]
            next_start_index = start_instances[i]
            prev_index = next_start_index - 1
            prev_time = self._raw_data.at[prev_index, self.CTRL_TIME]
            end_index = None
            for back_track in range(1, next_start_index - start_index - 1):
                t = self._raw_data.at[next_start_index - back_track, self.CTRL_TIME]
                if t != prev_time:
                    end_index = next_start_index - back_track
                    break
            self._traj_index_pairs.append((start_index, end_index))

        # Last trajectory
        start_index = start_instances[-1]
        last_index = len(self._raw_data) - 1
        prev_time = self._raw_data[self.CTRL_TIME].iloc[-1]
        end_index = None
        for back_track in range(last_index - start_index):
            if self._raw_data.at[last_index - back_track, self.CTRL_TIME] != prev_time:
                end_index = last_index - back_track
                break
        self._traj_index_pairs.append((start_index, end_index))

        # Find start/end of relevant data
        for i, val in enumerate(self._raw_data[self.CMD_CONFIG]):
            if len(val):
                self._data_start = i
                break
        for i in reversed(range(len(self._raw_data))):
            if len(self._raw_data.loc[i, self.CMD_CONFIG]):
                self._data_end = i
                break

    def get_raw_data(self):
        """Get the raw data extracted from binary file

        Returns:
            DataFrame: the binary file serialized to pandas.DataFrame object

        """
        return self._raw_data

    def get_raw_trajectory_data(self, index):
        """Get the raw data for a particular commanded trajectory

        This function is only valid if filter_data() was called.

        Args:
            index (int): the ith trajectory

        Returns:
            DataFrame: pandas.DataFrame object for the particular trajectory

        Raises:
            IndexError: If index exceeds the number of commanded trajectories

        """
        if (index is None) and (self._traj_index_pairs is None):
            self._raw_data[self._data_start:self._data_end]

        if index >= len(self._traj_index_pairs):
            raise IndexError("Index exceeds number of trajectories. Did you call filter_data()?")
        start, end = self._traj_index_pairs[index]

        return self._raw_data[start:end]

    def get_trajectory_data(self, index, delay_adjusted=False):
        """Get exact data for a particular commanded trajectory
        
        Exact data only contains timestamp information and position/velocity data. Position/velocity 
        data is also parsed into a numpy matrix format. This function is only valid if filter_data()
        was called.

        Args:
            index (int): the ith trajectory
            delay_adjusted (bool, optional): return data adjusted by the 
                control delay. Defaults to False.

        Returns:
            dict: dictionary for the particular trajectory, with numpy matrix 
                formated position/control data

        Raises:
            IndexError: If index exceeds the number of commanded trajectories
            RuntimeError: If estimate_delay() was not called by delay_adjusted 
                is requested
        
        """
        if index and index >= len(self._traj_index_pairs):
            raise IndexError("Index exceeds number of trajectories. Did you call filter_data()?")

        if delay_adjusted and not self._delay_estimated:
            raise RuntimeError("Delay not estimated. Could not get delay adjusted trajectory.")

        data_keys = [
            self.CMD_CONFIG, self.CMD_VEL, self.ROBOT_CONFIG, self.ROBOT_VEL, self.SYS_TIME,
            self.CTRL_TIME, self.CMD_TCP, self.ROBOT_TCP
        ]
        np_data = self._raw_data.values

        start = self._data_start
        end = self._data_end
        if index is not None:
            start, end = self._traj_index_pairs[index]

        ret_data = {}
        for k, v in self._column_ref.items():
            if k in data_keys:
                if k == self.SYS_TIME:
                    ret_data[k] = np_data[start:end, v]
                    continue
                if delay_adjusted and k in [self.ROBOT_CONFIG, self.ROBOT_VEL]:
                    # Correct for delays between the controller and robot
                    adjusted_start, adjusted_end = self._adjusted_delay_index[index]
                    ret_data[k] = np.vstack(np_data[adjusted_start:adjusted_end, v])
                else:
                    ret_data[k] = np.vstack(np_data[start:end, v])

        return ret_data

    def get_num_trajectories(self):
        """Returns total number of commanded trajectories in logs"""
        return len(self._traj_index_pairs)

    def get_delays(self):
        """Returns delay estimation for each commanded trajectory"""
        return self._delay_hist

    def get_update_rate(self):
        """Returns the update rate of the controller"""
        return self._update_rate

    def get_dof(self):
        """Returns the degrees of freedom of the robot"""
        return self._num_dof

    def get_pos_data(self, i=None, adjust_delay=False):
        """Get position data for a given trajectory

        Args:
            i (int, optional): The ith trajectory. Only valid if filter_data() was called. 
                Default is None (all trajectories).
            delay_adjusted (bool, optional): return data adjusted by the 
                control delay. Defaults to False.

        Returns:
            tuple: (time, commanded position, robot position)

        """
        trajectory_data = self.get_trajectory_data(i, adjust_delay)
        t = [dobj.to_datetime().timestamp() for dobj in trajectory_data[self.SYS_TIME]]
        p_c = trajectory_data[self.CMD_CONFIG]
        p_r = trajectory_data[self.ROBOT_CONFIG]

        return (t, p_c, p_r)

    def get_vel_data(self, i=None, adjust_delay=False):
        """Get velocity data for a given trajectory

        Args:
            i (int, optional): The ith trajectory. Only valid if filter_data() was called. 
                Default is None (all trajectories)
            delay_adjusted (bool, optional): return data adjusted by the 
                control delay. Defaults to False.

        Returns:
            tuple: (time, commanded velocity, robot velocity)

        """
        trajectory_data = self.get_trajectory_data(i, adjust_delay)
        t = [dobj.to_datetime().timestamp() for dobj in trajectory_data[self.SYS_TIME]]
        v_c = trajectory_data[self.CMD_VEL]
        v_r = trajectory_data[self.ROBOT_VEL]

        return (t, v_c, v_r)

    def get_accel_data(self, i=None, adjust_delay=False):
        """Get acceleration data for a given trajectory

        Args:
            i (int, optional): The ith trajectory. Only valid if filter_data() was called. 
                Default is None (all trajectories)
            delay_adjusted (bool, optional): return data adjusted by the 
                control delay. Defaults to False.

        Returns:
            tuple: (time, commanded acceleration, robot acceleration)

        """
        t, v_c, v_r = self.get_vel_data(i, adjust_delay)
        a_c = np.gradient(v_c, t, axis=0)
        a_r = np.gradient(v_r, t, axis=0)

        return (t, a_c, a_r)

    def get_jerk_data(self, i=None, adjust_delay=False):
        """Get jerk data for a given trajectory

        Args:
            i (int, optional): The ith trajectory. Default is None (all trajectories)
            delay_adjusted (bool, optional): return data adjusted by the 
                control delay. Defaults to False.

        Returns:
            tuple: (time, commanded jerk, robot jerk)

        """
        t, a_c, a_r = self.get_accel_data(i, adjust_delay)
        j_c = np.gradient(a_c, t, axis=0)
        j_r = np.gradient(a_r, t, axis=0)

        return (t, j_c, j_r)

    def get_tcp_pos(self, index=None, as_euler=True):
        if index and index >= len(self._traj_index_pairs):
            raise IndexError("Index exceeds number of trajectories. Did you call filter_data()?")

        data_keys = [self.SYS_TIME, self.CMD_TCP, self.ROBOT_TCP]
        np_data = self._raw_data.values

        start = self._data_start
        end = self._data_end
        if index is not None:
            start, end = self._traj_index_pairs[index]

        tcp_data = {}
        for k, v in self._column_ref.items():
            if k in data_keys:
                if k == self.SYS_TIME:
                    tcp_data[k] = np_data[start:end, v]
                    continue
                tcp_data[k] = np.vstack(np_data[start:end, v])

        t = [dobj.to_datetime().timestamp() for dobj in tcp_data[self.SYS_TIME]]

        cmd_pos = tcp_data[self.CMD_TCP][:, :3]
        robot_pos = tcp_data[self.ROBOT_TCP][:, :3]
        cmd_rot = tcp_data[self.CMD_TCP][:, 3:]
        robot_rot = tcp_data[self.ROBOT_TCP][:, 3:]

        if not as_euler:
            # Mask raw data for rotation calculations (cannot pass NaN values)
            mask_func = np.vectorize(lambda x: np.isnan(x).any())
            masked_cmd_rot = ~mask_func(cmd_rot)
            masked_robot_rot = ~mask_func(robot_rot)

            zero_pos = [0.0, 0.0, 0.0]
            processed_cmd_rot = np.where(masked_cmd_rot, cmd_rot, zero_pos)
            processed_robot_rot = np.where(masked_robot_rot, robot_rot, zero_pos)

            # Intrisic rotation is default
            cmd_rotations = Rotation.from_euler('XYZ', processed_cmd_rot)
            robot_rotations = Rotation.from_euler('XYZ', processed_robot_rot)

            cmd_rot = cmd_rotations.as_quat()
            robot_rot = robot_rotations.as_quat()

            # TODO (brian): if as_euler is False, we don't mask the rotation data,
            # so all of it gets printed. This is convenient for get_tcp_vel function,
            # but not as great as a stand alone if someone were to view the output.

        return (t, cmd_pos, cmd_rot, robot_pos, robot_rot)

    def get_tcp_vel(self, index=None):
        t, cmd_pos, cmd_rot, robot_pos, robot_rot = self.get_tcp_pos(index, False)

        cmd_rotations = Rotation.from_quat(cmd_rot)
        robot_rotations = Rotation.from_quat(robot_rot)
        cmd_spline = RotationSpline(t, cmd_rotations)
        robot_spline = RotationSpline(t, robot_rotations)

        cmd_vel = np.gradient(cmd_pos, t, axis=0)
        robot_vel = np.gradient(robot_pos, t, axis=0)

        cmd_angular_vel = cmd_spline(t, 1)
        robot_angular_vel = robot_spline(t, 1)

        # Mask angular vel results
        mask_func = np.vectorize(lambda x: np.isnan(x).any())
        masked_cmd_rot = ~mask_func(cmd_pos)
        masked_robot_rot = ~mask_func(robot_pos)

        nan_rot = [float("Nan")] * 3
        processed_cmd_angular_vel = np.where(masked_cmd_rot, cmd_angular_vel, nan_rot)
        processed_robot_angular_vel = np.where(masked_robot_rot, robot_angular_vel, nan_rot)

        return (t, cmd_vel, processed_cmd_angular_vel, robot_vel, processed_robot_angular_vel)

    def estimate_delay(self, max_delay=50):
        """Estimates the delay for each commanded trajectory
        
        The delay is estimated via brute force search over [0 max_delay] to 
        minimize the error between the command and the robot. Addtional 
        metadata is also extracted

        Args:
            max_delay (int, optional): The maximum delay for the search. 
                Default is 50.

        Returns:
            Array: a list of delay estimations, 1 for each trajectory

        """
        self._adjusted_delay_index = [None] * len(self._traj_index_pairs)
        self._delay_hist = []
        for traj_num, (b, e) in enumerate(self._traj_index_pairs):
            delay = 0
            error = float("inf")
            for i in range(max_delay):
                cmd_pos = self._raw_data[self.CMD_CONFIG].iloc[b:e].values

                # Shift robot data by delay
                adjusted_b = b + i
                adjusted_e = e + i
                if e < 0 or adjusted_e >= len(self._raw_data):
                    break
                robot_pos = self._raw_data[self.ROBOT_CONFIG].iloc[adjusted_b:adjusted_e].values

                # Calculate error
                total_error = 0
                for c, r in zip(cmd_pos, robot_pos):
                    total_error += np.sum(np.abs(c - r))
                avg_error = total_error / len(cmd_pos)

                # Set new delay
                if (avg_error < error):
                    self._adjusted_delay_index[traj_num] = (adjusted_b, adjusted_e)
                    error = avg_error
                    delay = i

            self._delay_hist.append(delay)

        self._delay_estimated = True
        return self._delay_hist

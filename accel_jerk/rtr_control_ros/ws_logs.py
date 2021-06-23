import pandas as pd
import numpy as np

from .log_constants import *
from .log_parse import read_ws_logs


class WSLogs(object):
    """The basic container for storing wireshark data

    WSLogs is used to parse wireshark logs dumped to .csv files. Ensure 
    that time data in wireshark is recorded as 'Seconds since 1970-01-01' 
    format (under View, Time Display Format)
    
    Attributes:
        ip_address (str): source ip address of interest
        jitter_cutoff (float): maximum acceptable network delay in seconds
        is_udp (bool, optional): True if communicati protocol is UDP, 
            False if TCP

    """
    _udp_key = "UDP"
    _tcp_key = "TCP"
    _time_key = "Time"
    _seq_key = "No."
    _of_key = "OverFlow"
    _prev_key = "PrevTime"
    _source_key = "Source"
    _protocol_key = "Protocol"

    def __init__(self, ip_address, jitter_cutoff, is_udp=True):
        self.ip_address = ip_address
        self.jitter_cutoff = jitter_cutoff

        self._raw_data = None
        self._filtered_data = None
        self._fdata_col_map = None
        self._traj_index_pairs = None
        if is_udp:
            self.protocol = self._udp_key
        else:
            self.protocol = self._tcp_key

    def extract_data(self, fname, robot_logs, cutoff=(0, 1)):
        """Parses wireshark .csv file and aligns data with corresponding RobotLogs
        
        Warning: wireshark time data must be in 'Seconds since 1970-01-01' format

        Args:
            fname (str): file path to wiresharek .csv data
            robot_logs (RobotLogs): corresponding robot logs data (must have 
                already extracted data)
            cutoff (tuple, optional): (start, end) percent of data to extract. 
                Example: (0.25, 0.75) reads all data between the first 25% and 
                first 75% of the data. Default is (0,1)
        
        """
        self._raw_data = read_ws_logs(fname, cutoff)

        self._filtered_data = self._raw_data[(self._raw_data[self._source_key] == self.ip_address) &
                                             (self._raw_data[self._protocol_key] == self.protocol)]

        # Network Filter
        diff = self._filtered_data[self._time_key].diff()
        mask = diff.le(self.jitter_cutoff) | diff.isnull()
        diff -= self.jitter_cutoff
        diff_series = diff.mask(mask, 0).rename(self._of_key)
        self._filtered_data = self._filtered_data.join(diff_series.to_frame())

        # Add a previous time column for easy access
        self._filtered_data[self._prev_key] = self._filtered_data[self._time_key].shift(periods=1)

        # Set sequence number index
        self._filtered_data.set_index(self._seq_key, inplace=True)

        # Name,index map for columns
        self._fdata_col_map = {col: i for i, col in enumerate(self._filtered_data.columns.tolist())}

        # Align wireshark data with robot logs
        self._align_robot_logs(robot_logs)

    def get_trajectory_data(self, index=None):
        """Get exact wireshark data for a particular commanded trajectory

        Args:
            index (int, optional): data for the ith trajectory. If None, then
                returns data for all trajectories

        Returns:
            DataFrame: wireshark data for the particular trajectory in 
                pandas.DataFrame object
        
        """
        if index and index >= len(self._traj_index_pairs):
            print("Index ({}) exceeds number of trajectories ({})".format(
                index, len(self._traj_index_pairs)))
            return None

        start = self._traj_index_pairs[0][0]
        end = self._traj_index_pairs[-1][-1]

        if index is not None:
            start, end = self._traj_index_pairs[index]

        return self._filtered_data.loc[start:end]

    def get_network_jitter_data(self, i=None):
        """Get network jitter for particular trajectory

        Args:
            i (int): data for ith trajectory. If None, then
                returns data for all trajectories

        Returns:
            list: list of tuples (start time, end time) when network jitter 
                occurred
        
        """
        if i and i >= len(self._traj_index_pairs):
            print("Index ({}) exceeds number of trajectories ({})".format(
                i, len(self._traj_index_pairs)))
            return []

        traj_data = self.get_trajectory_data(i)

        s_index = self._fdata_col_map[self._prev_key]
        e_index = self._fdata_col_map[self._time_key]
        o_index = self._fdata_col_map[self._of_key]
        region_indices = [(row[s_index], row[e_index])
                          for row in traj_data.itertuples(index=False)
                          if row[o_index] > 0]

        return region_indices

    def _align_robot_logs(self, robot_logs):
        num_traj = robot_logs.get_num_trajectories()

        # Get start/end times from robot_logs
        start_end_pairs = []
        for i in range(num_traj):
            traj_data = robot_logs.get_trajectory_data(i)
            start_time = traj_data[SYS_TIME][0]
            end_time = traj_data[SYS_TIME][-1]
            start_end_pairs.append((start_time.seconds + start_time.nanoseconds * 1e-9,
                                    end_time.seconds + end_time.nanoseconds * 1e-9))

        # Find closes start/end times in wireshark logs w.r.t. robot_logs
        self._traj_index_pairs = []
        for i, (b, e) in enumerate(start_end_pairs):
            start_index = self._filtered_data[self._time_key].ge(b).idxmax()
            end_index = self._filtered_data[self._time_key].ge(e).idxmax()
            self._traj_index_pairs.append((start_index, end_index))

        # Print message if wireshark logs don't cover all trajectories
        if len(self._traj_index_pairs) != num_traj:
            print("Missing wireshark data for trajectories: [{} {}]".format(
                num_traj - len(self._traj_index_pairs) - 1, num_traj - 1))

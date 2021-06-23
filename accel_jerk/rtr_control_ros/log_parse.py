import msgpack as mp
import numpy as np
import pandas as pd
import struct

from collections import Mapping
from pathlib import Path

VEC_DATA = "data_"


def parse_line(bin_file):
    """Parse a single msgpack line

    RTR Logging binary format: [signature, size, data]
    
        .--------.--------.--------------------.
        | 0x770F |  SIZE  |    ... DATA ...    |
        `--------'--------'--------------------'
           2 B      2 B        0 - 65535 B
    
    """
    head = bin_file.read(2)  # signature is 2 bytes
    if not head:
        # end of file
        return 0
    signature = struct.unpack('h', head)[0]
    if signature != 0x770F:
        raise Exception("Failed to read signature: {}, {}".format(signature, 0x770F))
    size = struct.unpack('h', bin_file.read(2))[0]  # size is 2 bytes

    return size


def read_control_logs(fname, cutoff=(0, 1)):
    """Parse msgpack serialized file to a pandas.DataFrame object"""
    if cutoff[0] < 0 or cutoff[1] > 1 or cutoff[1] <= cutoff[0]:
        raise ValueError("Invalid cutoff range. Must be [0 1]")

    fsize = float(Path(fname).stat().st_size)

    read_size = 0
    control_dict = {}
    with open(fname, "rb") as bin_file:
        # Dump beginning
        while read_size / fsize < cutoff[0]:
            size = parse_line(bin_file)
            read_size += size
            bin_file.read(size)

        size = parse_line(bin_file)
        while size > 0 and read_size / fsize <= cutoff[1]:
            msgpack_data = bin_file.read(size)
            msgpack_line = mp.unpackb(msgpack_data)
            for k, v in msgpack_line[1].items():
                if k not in control_dict:
                    control_dict[k] = [check_vec(v)]
                else:
                    control_dict[k].append(check_vec(v))
            size = parse_line(bin_file)
            read_size += size

    return pd.DataFrame(data=control_dict)


def read_datastream_logs(fname, cutoff=(0, 1)):
    """Parse msgpack-serialized file from DataStream to list"""
    if cutoff[0] < 0 or cutoff[1] > 1 or cutoff[1] <= cutoff[0]:
        raise ValueError("Invalid cutoff range. Must be [0 1]")

    fsize = float(Path(fname).stat().st_size)

    read_size = 0
    log_data = []
    with open(fname, "rb") as bin_file:
        # Dump beginning
        while read_size / fsize < cutoff[0]:
            size = parse_line(bin_file)
            read_size += size
            bin_file.read(size)

        size = parse_line(bin_file)
        while size > 0 and read_size / fsize <= cutoff[1]:
            msgpack_data = bin_file.read(size)
            msgpack_line = mp.unpackb(msgpack_data)
            log_data.append(msgpack_line)
            size = parse_line(bin_file)
            read_size += size

    return log_data


def check_vec(data):
    """Check if data is Vec() type and returns relevant data"""
    if isinstance(data, Mapping):
        if VEC_DATA in data:
            return np.array(data[VEC_DATA])

    return data


def read_ws_logs(fname, cutoff=(0, 1)):
    """Parse wireshark logs exported as .csv"""
    if cutoff[0] < 0 or cutoff[1] > 1 or cutoff[1] <= cutoff[0]:
        raise ValueError("Invalid cutoff range. Must be [0 1]")

    # Get number of lines in csv
    num_lines = 0
    with open(fname) as f:
        num_lines = sum(1 for line in f)  # generator expression

    start_cutoff = cutoff[0] * num_lines
    end_cutoff = cutoff[1] * num_lines
    skip_rows = [i for i in range(1, num_lines) if i < start_cutoff or i > end_cutoff]
    return pd.read_csv(fname, skiprows=skip_rows)

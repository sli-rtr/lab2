import matplotlib.pyplot as plt
import numpy as np
"""Control Plot Utilities"""


class PlotAttr(object):
    data = None
    label = ''
    line_attr = ''

    def __init__(self, d, l, a):
        self.data = d
        self.label = l
        self.line_attr = a


class YData(object):
    """Container storing a list of line data for a single plot"""

    def __init__(self):
        self._data = []

    def append(self, data, label, line_attr):
        """Append line data for plotting

        Args:
            data (numpy.ndarray): raw joint data
            label (str): label assigned to the line when plotted
            line_attr (str): line attribute (i.e. 'r:' is dotted red line)
        """
        if not isinstance(data, np.ndarray):
            raise Exception("Invalid data input")
        self._data.append(PlotAttr(data, label, line_attr))

    def __len__(self):
        return len(self._data)

    def __iter__(self):
        return self._data.__iter__

    def __getitem__(self, i):
        return self._data[i]


def basic_joint_plot(x,
                     y_data,
                     shade_regions=None,
                     highlight_data=None,
                     xlabel='x',
                     ylabel='y',
                     title='Plot'):
    """Basic unit for contructing a plot robot joint data

    Args:
        x (array): data for the x-axis
        y_data (YData): data for multiple lines
        shade_regions (PlotAttr, optional): data of index pairs, which creates 
            shaded regions
        highlight_indices (PlotAttr, optional): data of indexes, i, which get 
            highlighted in the plot position x[i]
        xlabel (str, optional): label for x-axis
        ylabel (str, optional): label for y-axis
        title (str, optional): title for plot
    """
    if not isinstance(y_data, YData):
        raise Exception("Invalid input for y. Not an instance of YData")

    num_dof = y_data[0].data.shape[1]
    fig = plt.figure()
    fig.suptitle(title)
    for i in range(num_dof):
        ax = plt.subplot(num_dof, 1, i + 1)
        for d in range(len(y_data)):
            y = y_data[d]
            plt.plot(x, y.data[:, i], y.line_attr, label=y.label)

        if shade_regions:
            for s, e in shade_regions.data:
                ax.axvspan(s,
                           e,
                           alpha=0.3,
                           color=shade_regions.line_attr,
                           label=shade_regions.label)

        if highlight_data:
            x_h = [x[i_h] for i_h in highlight_data.data]
            y_h = [y_data[0].data[i_h, i] for i_h in highlight_data.data]
            plt.plot(x_h, y_h, highlight_data.line_attr, label=highlight_data.label)
        plt.title("Joint: {}".format(i))
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.legend()


def basic_pose_plot(x, pos_data, rot_data, xlabel='x', ylabel=['m', 'rad'], title='Plot'):
    """Basic unit for contructing a plot robot joint data

    Args:
        x (array): data for the x-axis
        y_data (YData): data for multiple lines
        xlabel (str, optional): label for x-axis
        ylabel (str, optional): label for y-axis
        title (str, optional): title for plot
    """
    if not isinstance(pos_data, YData) or not isinstance(rot_data, YData):
        raise Exception("Invalid input for y. Not an instance of YData")

    if pos_data[0].data.shape[1] != 3 or rot_data[0].data.shape[1] != 3:
        print(pos_data[0].data)
        print("--")
        print(rot_data[0].data)
        raise Exception("Invalid pose input. Length != 3")

    titles = ["x", "y", "z", "r", "p", "y"]

    fig = plt.figure()
    fig.suptitle(title)
    for i in range(3):
        ax = plt.subplot(6, 1, i + 1)
        for d in range(len(pos_data)):
            y = pos_data[d]
            plt.plot(x, y.data[:, i], y.line_attr, label=y.label)

        plt.title("{}".format(titles[i]))
        plt.xlabel(xlabel)
        plt.ylabel(ylabel[0])
        plt.legend()

    for i in range(3, 6):
        ax = plt.subplot(6, 1, i + 1)
        for d in range(len(rot_data)):
            y = rot_data[d]
            plt.plot(x, y.data[:, i - 3], y.line_attr, label=y.label)

        plt.title("{}".format(titles[i]))
        plt.xlabel(xlabel)
        plt.ylabel(ylabel[1])
        plt.legend()

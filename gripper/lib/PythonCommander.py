#!/usr/bin/env python3

import socket
import select

class PythonCommander:
    """
    This class talks to the Realtime Controller over Socket
    It should implement all functions that our Realtime Controller supports
    Dec 11 2019 - Currently has parity with ICD version 1.4
    """
    DELIM = ","
    CMD_INDEX = 0
    CODE_INDEX = 1
    SEQ_INDEX = 2
    BAD_CODE = 1003  # this is the code for communication error
    INVALID_SEQ = -1

    # Constants
    DEFAULT_SEQ_NUM = 0
    MAX_BUFFER_SIZE = 4096
    SOCKET_POOL_SIZE = 25

    # String identifying ASCII Commands
    INIT_GROUP = 'InitGroup'
    BEGIN_OPERATION = 'BeginOperationMode'
    END_OPERATION = 'EndOperationMode'
    MOVE_TO_POSE = 'MoveToPose'
    MOVE_TO_HUB = 'MoveToHub'
    OFFROAD_TO_HUB = 'OffroadToHub'
    BLIND_MOVE = 'BlindMove'
    CANCEL_MOVE = 'CancelMove'
    TERMINATE_GROUP = 'TerminateGroup'
    CLEAR_FAULTS = 'ClearFaults'
    SET_INTERRUPT_BEHAVIOR = 'SetInterruptBehavior'
    ACQUIRE_CONTROL = 'AcquireControl'
    RELEASE_CONTROL = 'ReleaseControl'
    CHANGE_WORKSPACE = 'ChangeWorkState'
    GET_MODE = 'GetMode'

    # Return Codes
    SUCCESS = 0

    # 1XXX: Fatal dev errors
    RTR_SERVER_ERROR = 1001
    ROBOT_STATE_ERROR = 1002
    CONNECTION_ERROR = 1003
    FAILED_TO_CLEAR_FAULTS = 1004

    # 2XXX: user did something at the wrong state
    SERVER_NOT_IN_OPERATION_MODE = 2001
    CANCELED_BY_USER = 2002
    ARGUMENTS_INVALID = 2003
    SERVER_NOT_IN_CONFIG_MODE = 2004
    RAPID_SENSE_LOAD_ERROR = 2005
    BEGIN_OPERATION_MODE_TIMEOUT = 2006
    CANNOT_CALL_OFFROAD_WHILE_MOVING = 2007

    # 3XXX: Database/install errors
    DECONFLICTION_GROUP_NOT_FOUND = 3001
    PROJECT_NOT_FOUND = 3002
    WORK_STATE_NOT_FOUND = 3003
    PROJECT_LOAD_IN_PROGRESS = 3004
    PROJECT_CANNOT_RUN_SIMULATED_HW = 3005
    DECONFLICTION_GROUP_HAS_NO_PROJECTS = 3006

    # 4XXX: Planning errors
    NO_COLLISION_FREE_PATH = 4001
    NO_CONFIG_WITHIN_TOL_TO_GOAL = 4002
    NO_IK_SOLUTION_WITHIN_TOL_TO_ROADMAP = 4003
    HUB_NOT_IN_ROADMAP = 4004
    START_CONFIG_NOT_WITHIN_TOL_TO_ROADMAP = 4005
    REPLAN_ATTEMPTS_EXCEEDED = 4006

    # 5XXX: Rapidsense Errors
    CAMERA_ERROR = 5001

    # 6XXX: ?
    PATH_PLANNING_TIMEOUT = 6001

    def __init__(self, addr, port, socket_pool_size=None):
        """
        Constructor is called with an address (IP) and a port
        Parameters:
            addr (string): IP Address of the Realtime Controller
            port (int): Port number to connect to
            socket_pool_size (uint): number of concurrent socket connections (concurrent commands)
        """
        self._socket_pool_size = self.SOCKET_POOL_SIZE if (
            socket_pool_size is None) else socket_pool_size
        self._sockets = [socket.socket()
                         for _ in range(self._socket_pool_size)]
        self._reserved_sockets = {}
        self._addr = addr
        self._port = port

    @classmethod
    def _StringBuilder(cls, tokens):
        """Concatenates tokens in a delimited string"""
        tokens = [str(token) for token in tokens]
        cmd_str = cls.DELIM.join(tokens) + '\r\n'
        return cmd_str

    @classmethod
    def _SplitToTokens(cls, data):
        """Splits a string by delimiter into tokens. Cuts carriage returns on last token."""
        tokens = data.split(cls.DELIM)
        tokens[-1] = tokens[-1][:-2]
        return tokens

    @classmethod
    def _GetCode(cls, data):
        """Grabs the value of the return code out of a return strings"""
        tokens = cls._SplitToTokens(data)
        if len(tokens) > cls.CODE_INDEX:
            return int(tokens[cls.CODE_INDEX])
        else:
            return cls.BAD_CODE

    @classmethod
    def _GetSeq(cls, data):
        """Grabs the value of the sequence number out of a return strings"""
        tokens = cls._SplitToTokens(data)
        if len(tokens) > cls.SEQ_INDEX:
            return tokens[cls.SEQ_INDEX]
        else:
            return cls.DEFAULT_SEQ_NUM

    def _GetCodeSeq(self, data):
        return (self._GetCode(data), self._GetSeq(data))

    def _GetSocket(self):
        return self._sockets.pop()

    def _ReturnSocket(self, socket):
        self._sockets.append(socket)

    def _SendAndRecv(self, cmd_tokens, has_delayed_response):
        """
        Combines tokens into ascii string, sends it, and waits for a reply.
        If has_delayed_response is set, caches the socket into _reserved_sockets dict
        Parameters:
            cmd_tokens (list) - all string tokens (in order) to be sent over with comma seperation
            has_delayed_response (bool) - indicates whether to expect a second response for call
        Return:
            (uint, int, string) - The parsed result code, the seq number (INVALID_SEQ if not
                                  applicable) and raw data string are returned as a tuple
        """
        socket = self._GetSocket()  # get a socket from available list

        cmd_str = PythonCommander._StringBuilder(
            cmd_tokens)  # build the cmd str
        print(f'Sending: {cmd_str}')
        socket.sendall(cmd_str.encode())  # send our cmd string
        # recv response
        data = socket.recv(PythonCommander.MAX_BUFFER_SIZE).decode("utf-8")
        print(data)

        # Parse the code and set seq to INVALID
        code = self._GetCode(data)
        seq = self.INVALID_SEQ

        # If we got a SUCCESS response and have a delayed response, parse seq and reserve socket
        if has_delayed_response and (code == self.SUCCESS):
            seq = self._GetSeq(data)
            self._reserved_sockets[seq] = socket

        # Otherwise, just return the socket
        else:
            self._ReturnSocket(socket)

        # return the code, seq and data
        return (code, seq, data)

    def _Call(self, tokens, has_delayed_response):
        """
        Sends and receives ascii command using tokens, catching exceptions if they occur.
        Returns either a uint of just the code, or a tuple of code, seq_num depending on
        has_delayed_response flag
        """
        try:
            code, seq, data = self._SendAndRecv(tokens, has_delayed_response)
            # Three cases:
            # Case 1: 'Normal' command - just returns a code
            # Case 2: 'Move' command - returns a code and sequence number
            # Case 3: 'State' command - returns a code and string
            data_list = data.split(',')

            # Remove leading and ending space from strings
            for count,entry in enumerate(data_list):
                data_list[count] = entry.strip()

            assert(len(data_list) > 0)
            if has_delayed_response:
                return (code, seq)
            elif data_list[0] == "GetMode":
                assert(len(data_list) == 3)
                return (code, data_list[2])
            else:
                return code
        except Exception as e:
            print(
                "Got an exception while sending ascii command {}, [{}]".format(tokens, e))
            return self.BAD_CODE

    def _Connect(self):
        """
        Connects to the addr / port stored in self (catches exceptions)
        """
        new_sockets = [socket.socket() for _ in range(self._socket_pool_size)]
        is_connected = [True for _ in range(self._socket_pool_size)]
        for ii in range(self._socket_pool_size):
            try:
                new_sockets[ii].connect((self._addr, self._port))
            except Exception as e:
                is_connected[ii] = False
                print(
                    'Got exception in PythonCommander Init [{}]'.format(e))
                continue

        # remove any sockets which failed to connect
        if not len(is_connected) == len(new_sockets):
            print("Error unequal list lengths")
        self._sockets = []
        for idx in range(len(new_sockets)):
            if is_connected[idx]:
                self._sockets.append(new_sockets[idx])
        return len(self._sockets)

    def Reconnect(self, addr=None, port=None, socket_pool_size=None):
        """
        Deletes all the sockets, creates knew ones and reconnects. If an addr and port is specified
        (both must be), then it'll use the new pair, otherwise it'll use the latest one in cache.
        If a new socket pool size is specified, it'll create that many sockets for the pool,
        otherwise it'll use the previously cached value
        """
        del self._reserved_sockets
        del self._sockets

        if (not (addr == None)) and (not (port == None)):
            self._addr = addr
            self._port = port

        self._reserved_sockets = {}
        self._socket_pool_size = self._socket_pool_size if (
            socket_pool_size is None) else socket_pool_size
        return self._Connect() > 0  # won't throw

    def Setup(self, group_name, project_name):
        """
        Caches the group name and project name.
        All methods to call commands no longer need those arguments.
        This should be called before any other method is called if using project specific commands.
        If only using group level commands (eg BeginOperation), then this is not required
        """
        self._group_name = group_name
        self._project_name = project_name
        return self.Reconnect()

    def GetMode(self):
        """
        Sends the GetMode command which queries the Realtime Controller's state
        Returns:
            unsigned int: return code, 0 means success
            str: string representation of the current Realtime Controller state
        """
        tokens = [self.GET_MODE]
        return self._Call(tokens, False)

    def BeginOperation(self):
        """
        Sends the BeginOperation command which attempts CONFIG -> RUN transition.
        Returns:
            unsigned int: return code, 0 means success
        """
        tokens = [self.BEGIN_OPERATION]
        return self._Call(tokens, False)

    def EndOperation(self):
        """
        Sends the EndOperation command which attempts RUN -> CONFIG transition.
        Returns:
            unsigned int: return code, 0 means success
        """
        tokens = [self.END_OPERATION]
        return self._Call(tokens, False)

    def ClearFaults(self):
        """
        Sends the ClearFaults command which attempts FAULT -> CONFIG transition.
        Returns:
            unsigned int: return code, 0 means success
        """
        tokens = [self.CLEAR_FAULTS]
        return self._Call(tokens, False)

    def InitGroup(self, workspace, group_name=None, project_name=None):
        """
        Sends the InitGroup command with initial workspace. Must call Setup(...) first, or
        specify both the group_name and project_name args
        Parameters:
            workspace (string): Initial workspace
            group_name (string) - name of the deconfliction group to terminate
            project_name (string): name of the robot project this should affect
        Returns:
            unsigned int: return code, 0 means success
        """
        group_name = self._group_name if (group_name is None) else group_name
        project_name = self._project_name if (
            project_name is None) else project_name
        tokens = [self.INIT_GROUP, group_name, project_name, workspace]
        return self._Call(tokens, False)

    def TerminateGroup(self, group_name=None):
        """
        Sends the TerminateGroup command which unloads a deconfliction group.
        Must call Setup(...) first, or specify the group_name parameter
        Parameters:
            group_name (string) - name of the deconfliction group to terminate
        Returns:
            unsigned int: return code, 0 means success
        """
        group_name = self._group_name if (group_name is None) else group_name
        tokens = [self.TERMINATE_GROUP, group_name]
        return self._Call(tokens, False)

    def WaitForMove(self, seq_num, timeout=30.0):
        """
        Waits for MoveResult to be received
        Parameters:
            seq_num (uint): sequence number to who's delayed response to wait for
            timeout (double): timeout in seconds, after which to give up
        Returns:
            uint (or None): return code, 0 means success, None if seq_num isn't found in dict of
                            self._reserved_sockets or timeout
        """
        # If the seq_num specified doesn't exist, just return None
        if not (seq_num in self._reserved_sockets):
            return self.ARGUMENTS_INVALID

        timed_out = False
        # grab the socket from reserved list
        socket = self._reserved_sockets.pop(seq_num)
        socket.settimeout(timeout)  # set the timeout
        try:
            data = socket.recv(
                PythonCommander.MAX_BUFFER_SIZE)  # try to recv

            # In python 3 strings need to be converted from bytes to unicode
            data = data.decode('utf_8')
            print(data)

        except Exception as e:
            print("Timed out waiting for delayed response, got {}".format(e))
            timed_out = True

        finally:
            socket.settimeout(None)  # reset the timeout
            self._ReturnSocket(socket)  # return the socket to the pool

        return self.BAD_CODE if timed_out else self._GetCode(data)

    def SetInterruptBehavior(self, replan_attempts, timeout, project_name=None):
        """
        Sets the planning parameters. Must call Setup(...) first.
        Parameters:
            replan_attempts (int): Number of times Realtime Controller will attempt to replan if blocked
            timeout (float): how long to attempt to find a plan
            project_name (string): name of the robot project this should affect
        Returns:
            unsigned int: return code, 0 means success
        """
        project_name = self._project_name if (
            project_name is None) else project_name
        tokens = [self.SET_INTERRUPT_BEHAVIOR,
                  project_name, replan_attempts, timeout]
        return self._Call(tokens, False)

    def MoveToHub(self, workspace, hub, speed, corner_smoothing, project_name=None):
        """
        Sends a Move to Hub command. Must call Setup(...) first.
        Parameters:
            workspace (string): workspace to make the move in
            hub (string): name of hub to move to
            speed (float): relative speed of motion (between 0 and 1)
            project_name (string): name of the robot project this should affect
        Returns:
            (uint, uint): (return code - 0 means succes, sequence number - used for WaitForMove)
        """
        project_name = self._project_name if (
            project_name is None) else project_name
        tokens = [self.MOVE_TO_HUB, project_name, workspace, hub, speed, 0.0]
        return self._Call(tokens, True)

    def MoveToPose(self, workspace, x, y, z, rx, ry, rz, tol_x, tol_y, tol_z, tol_rx, tol_ry,
                   tol_rz, complete_move, complete_move_type, speed, project_name=None):
        """
        Sends a Move to Pose command. Must call Setup(...) first.
        Moves robot to specified cartesian coordinate if it's within tolerance to roadmap
        Parameters:
            workspace (string): workspace to make the move in
            x,y,z,rz,ry,rz (float): desired cartesian coordinate of arm tool
            tol_x, tol_y, tol_z, tol_rx, tol_ry, tol_rz (float): desired tolerance
            complete_move (bool): whether or not to go to exact point or closest point on roadmap
            complete_move_type (int): what kind of interpolation to use for last mile
            speed (float): relative speed of motion (between 0 and 1)
            project_name (string): name of the robot project this should affect
        Returns:
            (uint, uint): (return code - 0 means succes, sequence number - used for WaitForMove)
        """
        project_name = self._project_name if (
            project_name is None) else project_name
        tokens = [
            self.MOVE_TO_POSE, project_name, workspace,
            x, y, z, rx, ry, rz,
            tol_x, tol_y, tol_z, tol_rx, tol_ry, tol_rz,
            complete_move, complete_move_type, speed, 0.0
        ]
        return self._Call(tokens, True)

    def BlindMove(self, workspace, x, y, z, rx, ry, rz, move_type, speed, ignore_all_collisions=False, project_name=None):
        """
        Sends a Blind Move command. Must call Setup(...) first.
        Moves robot off roadmap to exact point with static collision checking only
        Parameters:
            workspace (string): workspace to make the move in
            x,y,z,rz,ry,rz (float): desired cartesian coordinate of arm tool
            move_type (int): interpolation type of the motion
            speed (float): relative speed of motion (between 0 and 1)
            ignore_all_collisions (bool): disables all collision checking when asserted True
            project_name (string): name of the robot project this should affect
        Returns:
            (uint, uint): (return code - 0 means succes, sequence number - used for WaitForMove)
        """
        project_name = self._project_name if (
            project_name is None) else project_name
        tokens = [self.BLIND_MOVE, project_name, workspace, x, y,
                  z, rx, ry, rz, move_type, speed, ignore_all_collisions]
        return self._Call(tokens, True)

    def OffroadToHub(self, workspace, hub, opt, timeout, fallback_to_nominal, speed, project_name=None):
        """
        Sends a Blind Move command. Must call Setup(...) first.
        Moves robot from off roadmap to specified hub with static collision checking only.
        Parameters:
            workspace (string): workspace to make the move in
            hub (string): name of hub to move to
            opt (string): Trades between quality of planning ('low', 'medium' or 'high') and time
            timeout (float): how long to try and find a plan for
            fallback_to_nominal: true if Offroad is allowed to plan with non-dilated meshes
            project_name (string): name of the robot project this should affect
        Returns:
            (uint, uint): (return code - 0 means succes, sequence number - used for WaitForMove)
        """
        project_name = self._project_name if (
            project_name is None) else project_name
        tokens = [self.OFFROAD_TO_HUB, project_name,
                  workspace, hub, opt, timeout, fallback_to_nominal, speed]
        return self._Call(tokens, True)

    def AcquireControl(self, workspace, project_name=None):
        """
        Sends an Acquire Control command. Must call Setup(...) first.
        Command to indicate the user is done with an operation that required RTR
        to release external control of the robot, and the RTR controller should
        resume control, with the state of the robot in the workspace indicated.
        Parameters:
            workspace (string): workspace of the robot when acquiring control
            project_name (string): name of the robot project this should affect
        Returns:
            unsigned inst: return code 0 means success
        """
        project_name = self._project_name if (
            project_name is None) else project_name
        tokens = [self.ACQUIRE_CONTROL, project_name, workspace]
        return self._Call(tokens, False)

    def ReleaseControl(self, workspace, project_name=None):
        """
        Sends a Release Control command. Must call Setup(...) first.
        Command to indicate the user would like to take temporary control of the
        robot to get/set IOs, or execute a portion of their robot program that is
        not suitable for RTR control at this point.
        Parameters:
            workspace (string): workspace of the robot for while control has been release
            project_name (string): name of the robot project this should affect
        Returns:
            unsigned inst: return code 0 means success
        """
        project_name = self._project_name if (
            project_name is None) else project_name
        tokens = [self.RELEASE_CONTROL, project_name, workspace]
        return self._Call(tokens, False)

    def CancelMove(self, workspace, project_name=None):
        """
        Sends a Cancel Move command. Must call Setup(...) first.
        This command tells the RTR Motion Planning Server to abort planning and
        motion for the specified robot, and informs the Motion Planning Server
        what state the robot is in after the cancellation.
        Parameters:
            workspace (string): desired workspace of the robot following cancellation
            project_name (string): name of the robot project this should affect
        Returns:
            unsigned inst: return code 0 means success
        """
        project_name = self._project_name if (
            project_name is None) else project_name
        tokens = [self.CANCEL_MOVE, project_name, workspace]
        return self._Call(tokens, False)

    def ChangeWorkspace(self, workspace, project_name=None):
        """
        Sends a Change Workspace command. Must call Setup(...) first.
        Command to change the active workspace from the current one to the one specified
        Parameters:
            workspace (string): desired workspace
            project_name (string): name of the robot project this should affect
        Returns:
            unsigned inst: return code 0 means success
        """
        project_name = self._project_name if (
            project_name is None) else project_name
        tokens = [self.CHANGE_WORKSPACE, project_name, workspace]
        return self._Call(tokens, False)

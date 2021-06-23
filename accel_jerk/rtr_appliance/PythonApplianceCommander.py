#!/usr/bin/env python3

import socket
import select
# import rospy


class PythonApplianceCommander:
    """
    This class talks to the Appliance over Socket
    It should implement all functions that our Appliance supports
    It is an alternative to the CPP based Appliance Commander

    Dec 11 2019 - Currently has parity with ICD version 1.4
    """
    DELIM = ","
    CMD_INDEX = 0
    CODE_INDEX = 1
    SEQ_INDEX = 3
    BAD_CODE = -1
    INVALID_SEQ = -1

    # Constants
    DEFAULT_SEQ_NUM = 0
    MAX_BUFFER_SIZE = 4096
    SOCKET_POOL_SIZE = 25

    # String identifying ASCII Commands
    LOAD_GROUP = 'LoadGroup'
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
    CHOREOGRAPHY_INSTALL = 'ChoreographyInstall'
    CHOREOGRAPHY_UNINSTALL = 'ChoreographyUninstall'
    CHOREOGRAPHY_LOAD = 'ChoreographyLoad'
    CHOREOGRAPHY_UNLOAD = 'ChoreographyUnload'
    CHOREOGRAPHY_STAGE = 'ChoreographyStage'
    CHOREOGRAPHY_UNSTAGE = 'ChoreographyUnstage'
    CHOREOGRAPHY_RUN = 'ChoreographyRun'
    CHOREOGRAPHY_CANCEL = 'ChoreographyCancel'
    CHOREOGRAPHY_STEP = 'ChoreographyStep'

    # Return Codes
    SUCCESS = 0

    # 1XXX: Fatal dev errors
    RTR_SERVER_ERROR = 1001
    ROBOT_STATE_ERROR = 1002
    CONNECTION_ERROR = 1003
    FAILED_TO_CLEAR_FAULTS = 1004
    MPA_ERROR = 1005
    EULA_NOT_ACCEPTED = 1006

    # 2XXX: user did something at the wrong state
    SERVER_NOT_IN_OPERATION_MODE = 2001
    CANCELED = 2002
    ARGUMENTS_INVALID = 2003
    SERVER_NOT_IN_CONFIG_MODE = 2004
    RAPID_SENSE_LOAD_ERROR = 2005
    BEGIN_OPERATION_MODE_TIMEOUT = 2006
    CANNOT_CALL_OFFROAD_WHILE_MOVING = 2007
    RAPID_SENSE_NOT_IN_OPERATION_MODE = 2008
    CHANGE_WORK_STATE_INVALID_WHILE_IN_MOTION = 2009
    PROJECT_ALREADY_EXISTS = 2010
    PROJECT_IN_HANDOFF_MODE = 2011
    PROJECT_NOT_IN_HANDOFF_MODE = 2012
    CANNOT_CALL_MOVE_DURING_OFFROAD_MOVE = 2013
    HUB_NOT_IN_PROJECT = 2016

    # 3XXX: Database/install errors
    DECONFLICTION_GROUP_NOT_FOUND = 3001
    PROJECT_NOT_FOUND = 3002
    WORK_STATE_NOT_FOUND = 3003
    PROJECT_LOAD_IN_PROGRESS = 3004
    PROJECT_CANNOT_RUN_SIMULATED_HW = 3005
    DECONFLICTION_GROUP_HAS_NO_PROJECTS = 3006
    OTHER_DECONFLICTION_GROUP_ALREADY_LOADED = 3008
    DECONF_GROUP_NOT_LOADED = 3009
    VOXEL_REGIONS_DONT_MATCH = 3010
    DECONF_GROUP_PROJECT_SAME_IP_PORT = 3011
    ROBOT_MODELS_ERROR = 3012
    DECONF_GROUP_EXCEEDED_MAX_ROBOTS = 3013
    OLD_VERSION_PROJECT = 3014

    # 4XXX: Planning errors
    NO_COLLISION_FREE_PATH = 4001
    NO_CONFIG_WITHIN_TOL_TO_GOAL = 4002
    NO_IK_SOLUTION_WITHIN_TOL_TO_ROADMAP = 4003
    HUB_NOT_REACHABLE_IN_WORKSTATE = 4004
    START_CONFIG_NOT_WITHIN_TOL_TO_ROADMAP = 4005
    REPLAN_ATTEMPTS_EXCEEDED = 4006
    ALREADY_EXECUTING_A_GOAL = 4007
    OG_READ_FAILURE = 4008
    PATH_PLANNING_TIMEOUT = 4009
    START_GOAL_NOT_CONNECTED = 4010
    ROBOT_IN_COLLISION = 4011
    COMPLETE_MOVE_STATIC_COLLISION = 4013
    ENTER_ROADMAP_STATIC_COLLISION = 4013
    WAYPOINT_SPACING = 4014

    # 5XXX: Rapidsense Errors
    CAMERA_ERROR = 5001
    VOXEL_UPDATE_ERROR = 5002

    # 7XXX: Controller/trajectory errors
    COMMAND_CONFLICT = 7001
    TRAJECTORY_INVALID_IK = 7002
    ACQUIRE_RELEASE_CONTROL_FAILURE = 7003

    def __init__(self, addr, port, socket_pool_size=None):
        """
        Constructor is called with an address (IP) and a port

        Parameters:
            addr (string): IP Address of the Appliance
            port (int): Port number to connect to
            socket_pool_size (uint): number of concurrent socket connections (concurrent commands)
        """
        self._socket_pool_size = self.SOCKET_POOL_SIZE if (
            socket_pool_size is None) else socket_pool_size
        self._sockets = [socket.socket() for _ in range(self._socket_pool_size)]
        self._choreo_socket = socket.socket()
        self._reserved_sockets = {}
        self._preserved_respond = {}  # for RAPID-8134, store the delay response received with ack
        self._addr = addr
        self._port = port

    @classmethod
    def _StringBuilder(cls, tokens):
        """Concatenates tokens in a delimited string"""
        tokens = [str(token) for token in tokens]
        cmd_str = cls.DELIM.join(tokens) + '\r\n'
        print('built command: ', cmd_str)
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

    def _SendAndRecvHelper(self, cmd_tokens, socket):
        cmd_str = PythonApplianceCommander._StringBuilder(cmd_tokens)  # build the cmd str
        socket.sendall(cmd_str.encode('utf-8'))  # send our cmd string
        data = socket.recv(PythonApplianceCommander.MAX_BUFFER_SIZE).decode('utf-8')
        data = str(data)  # python2 decode will return type unicode. need str convertion

        # Parse the code and set seq to INVALID
        code = self._GetCode(data)
        seq = self.INVALID_SEQ

        return code, data

    def _SendAndRecv(self, cmd_tokens, reserve_socket):
        """
        Combines tokens into ascii string, sends it, and waits for a reply.
        If reserve_socket is set, socket is returned to caller rather than returned to pool
        Parameters:
            cmd_tokens (list) - all string tokens (in order) to be sent over with comma seperation
            reserve_socket (bool) - indicates whether socket should be returned to caller for additional responses

        Return:
            (uint, string, socket) - The parsed result code, raw data string, and the socket if applicable are returned as a tuple
        """
        socket = self._GetSocket()  # get a socket from available list
        code, data = self._SendAndRecvHelper(cmd_tokens, socket)

        # If we got a SUCCESS response and have a delayed response, parse seq and reserve socket
        if reserve_socket and (code == self.SUCCESS):
            return (code, data, socket)

        # Otherwise, return the socket to the pool
        self._ReturnSocket(socket)

        # return the code and data to the caller
        return (code, data, None)

    def _Call(self, tokens, reserve_socket):
        """
        Sends and receives ascii command using tokens, catching exceptions if they occur.
        Returns a tuple of code, data, socket.
        socket will be None if reserve_socket is false or code is not success
        """
        try:
            return self._SendAndRecv(tokens, reserve_socket)
        except Exception as e:
            print("Got an exception while sending ascii command {}, [{}]".format(tokens, e))
            return (self.BAD_CODE, None, None)

    def _CallMove(self, tokens):
        """
        Helper function to perform common logic when sending move calls
        """
        (code, data, socket) = self._Call(tokens, True)
        seq_num = self.INVALID_SEQ

        if socket != None:
            seq_num = self._GetSeq(data)
            #TODO(Leo Chen RAPID-8134) temp fix for mixed ack and delay resp
            # Assumes when the socket.recv in _SendAndRecvHelper got more then one message,
            # There are no more then 2 messages concatenated
            # (at most a ACK response concatinated by a delayed response)
            if data.count("\r\n") > 1:
                lines = data.splitlines(True)  # Python2 ver doesn't have keyword keepends
                seq_num = self._GetSeq(lines[0])
                self._preserved_respond[seq_num] = lines[1]
                print("Catched a delay response mixed with ack. Resposne stored.")

            self._reserved_sockets[seq_num] = socket

        return (code, seq_num)

    def _CallCode(self, tokens):
        """
        Helper function for callers who only want the return code
        """
        return self._Call(tokens, False)[0]

    # def _Connect(self):
    #     """
    #     Connects to the addr / port stored in self (catches exceptions)
    #     """
    #     self._choreo_socket = socket.socket()
    #     try:
    #         self._choreo_socket.connect((self._addr, self._port))
    #     except Exception as e:
    #         rospy.logerr('Got exception while connecting choreo socket [{}]'.format(e))
    #         return 0
    #
    #     new_sockets = [socket.socket() for _ in range(self._socket_pool_size)]
    #     is_connected = [True for _ in range(self._socket_pool_size)]
    #     for ii in range(self._socket_pool_size):
    #         try:
    #             new_sockets[ii].connect((self._addr, self._port))
    #         except Exception as e:
    #             is_connected[ii] = False
    #             rospy.logerr('Got exception in PythonApplianceCommander Init [{}]'.format(e))
    #             continue
    #
    #     # remove any sockets which failed to connect
    #     if not len(is_connected) == len(new_sockets):
    #         print("Error unequal list lengths")
    #     self._sockets = []
    #     for idx in range(len(new_sockets)):
    #         if is_connected[idx]:
    #             self._sockets.append(new_sockets[idx])
    #     return len(self._sockets)

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

    def _WaitForResult(self, socket_id, timeout):
        """
        Waits for a message from the reserved socket specified by socket_id
        Returns:
            unsigned int: return code, 0 means success
        """
        if not (socket_id in self._reserved_sockets):
            return self.ARGUMENTS_INVALID

        timed_out = False
        # grab the socket from reserved list
        socket = self._reserved_sockets.pop(socket_id)
        #TODO(Leo Chen RAPID-8134) temp fix for mixed ack and delay resp
        if socket_id in self._preserved_respond:
            print("Got delayed response from previous preserve data")
            self._ReturnSocket(socket)
            data = self._preserved_respond.pop(socket_id)
            return self._GetCode(data)

        socket.settimeout(timeout)  # set the timeout
        try:
            data = socket.recv(PythonApplianceCommander.MAX_BUFFER_SIZE).decode('utf-8')

        except Exception as e:
            print("Timed out waiting for delayed response, got {}".format(e))
            timed_out = True

        finally:
            socket.settimeout(None)  # reset the timeout
            self._ReturnSocket(socket)  # return the socket to the pool

        return self.BAD_CODE if timed_out else self._GetCode(data)

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
            str: string representation of the current appliance state
        """
        tokens = [self.GET_MODE]
        (code, data, _) = self._Call(tokens, False)
        data_list = data.split(',')
        assert (len(data_list) == 3)
        return (code, data_list[2])

    def BeginOperation(self):
        """
        Sends the BeginOperation command which attempts CONFIG -> OPERATION transition.

        Returns:
            unsigned int: return code, 0 means success
        """
        tokens = [self.BEGIN_OPERATION]
        return self._CallCode(tokens)

    def EndOperation(self):
        """
        Sends the EndOperation command which attempts OPERATION -> CONFIG transition.

        Returns:
            unsigned int: return code, 0 means success
        """
        tokens = [self.END_OPERATION]
        return self._CallCode(tokens)

    def ClearFaults(self):
        """
        Sends the ClearFaults command which attempts FAULT -> CONFIG transition.

        Returns:
            unsigned int: return code, 0 means success
        """
        tokens = [self.CLEAR_FAULTS]
        return self._CallCode(tokens)

    def LoadGroup(self, group_name=None):
        """
        Sends the LoadGroup command

        Parameters:
            workspace (string): Initial workspace

        Returns:
            unsigned int: return code, 0 means success
        """
        group_name = self._group_name if (group_name is None) else group_name
        tokens = [self.LOAD_GROUP, group_name]
        (code, data, socket) = self._Call(tokens, True)
        if socket != None:
            self._reserved_sockets["LoadGroup"] = socket

        return code

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
        project_name = self._project_name if (project_name is None) else project_name
        tokens = [self.INIT_GROUP, group_name, project_name, workspace]
        return self._CallCode(tokens)

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
        return self._CallCode(tokens)

    def WaitForLoadGroup(self, timeout=1800):
        """
        Waits for LoadResult to be received

        Parameters:
            timeout (double): timeout in seconds, after which to give up

        Returns:
            uint (or None): return code, 0 means success, None if Load command isn't found in dict of
                            self._reserved_sockets or timeout
        """
        return self._WaitForResult("LoadGroup", timeout)

    def WaitForMove(self, seq_num, timeout=240.0):
        """
        Waits for MoveResult to be received

        Parameters:
            seq_num (uint): sequence number to who's delayed response to wait for
            timeout (double): timeout in seconds, after which to give up

        Returns:
            uint (or None): return code, 0 means success, None if seq_num isn't found in dict of
                            self._reserved_sockets or timeout
        """
        # If the seq_num specified doesn't exist, return arguments invalid
        return self._WaitForResult(seq_num, timeout)

    def IterableWaitForMove(self, seq_num, timeout=1.0):
        """
        Waits for MoveResult to be received.
        If no MoveResult is received before timeout, the execution continue as normal.

        Parameters:
            seq_num (uint): sequence number to who's delayed response to wait for
            timeout (double): timeout in seconds, after which execution continue

        Returns:
            unsigned int: return code, 0 means success,
                            ARGUMENTS_INVALID if seq_num isn't found in dict of self._reserved_sockets
                            BAD_CODE it timeout is reached before receiving the MoveResult
        """
        if not (seq_num in self._reserved_sockets):
            return self.ARGUMENTS_INVALID

        timed_out = False
        # copy socket from reserved list
        socket = self._reserved_sockets[seq_num]
        socket.settimeout(timeout)  # set the timeout
        data = None
        try:
            data = socket.recv(self.MAX_BUFFER_SIZE).decode()

        except Exception as e:
            timed_out = True

        except:
            print("An error exception occurred, , got {}".format(sys.exc_info()[0]))

        else:
            self._ReturnSocket(self._reserved_sockets.pop(seq_num))  # return the socket to the pool

        return None if timed_out else self._GetCode(data)

    def SetInterruptBehavior(self, replan_attempts, timeout, project_name=None):
        """
        Sets the planning parameters. Must call Setup(...) first.

        Parameters:
            replan_attempts (int): Number of times Appliance will attempt to replan if blocked
            timeout (float): how long to attempt to find a plan
            project_name (string): name of the robot project this should affect

        Returns:
            unsigned int: return code, 0 means success
        """
        project_name = self._project_name if (project_name is None) else project_name
        tokens = [self.SET_INTERRUPT_BEHAVIOR, project_name, replan_attempts, timeout]
        return self._CallCode(tokens)

    def MoveToHub(self, workspace, hub, speed, smooth_dist=0, project_name=None):
        """
        Sends a Move to Hub command. Must call Setup(...) first.

        Parameters:
            workspace (string): workspace to make the move in
            hub (string): name of hub to move to
            speed (float): relative speed of motion (between 0 and 1)
            smooth_dist (float): decides smoothing start and end points
            project_name (string): name of the robot project this should affect

        Returns:
            (uint, uint): (return code - 0 means succes, sequence number - used for WaitForMove)
        """
        project_name = self._project_name if (project_name is None) else project_name
        tokens = [self.MOVE_TO_HUB, project_name, workspace, hub, speed, smooth_dist]
        return self._CallMove(tokens)

    def MoveToPose(self,
                   workspace,
                   x,
                   y,
                   z,
                   rx,
                   ry,
                   rz,
                   tol_x,
                   tol_y,
                   tol_z,
                   tol_rx,
                   tol_ry,
                   tol_rz,
                   complete_move,
                   complete_move_type,
                   speed,
                   smooth_dist=0,
                   project_name=None):
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
            smooth_dist (float): decides smoothing start and end points
            project_name (string): name of the robot project this should affect

        Returns:
            (uint, uint): (return code - 0 means succes, sequence number - used for WaitForMove)
        """
        project_name = self._project_name if (project_name is None) else project_name
        tokens = [
            self.MOVE_TO_POSE, project_name, workspace, x, y, z, rx, ry, rz, tol_x, tol_y, tol_z,
            tol_rx, tol_ry, tol_rz, complete_move, complete_move_type, speed, smooth_dist
        ]
        return self._CallMove(tokens)

    def BlindMove(self,
                  workspace,
                  x,
                  y,
                  z,
                  rx,
                  ry,
                  rz,
                  move_type,
                  speed,
                  ignore_all_collisions=False,
                  project_name=None):
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
        project_name = self._project_name if (project_name is None) else project_name
        tokens = [
            self.BLIND_MOVE, project_name, workspace, x, y, z, rx, ry, rz, move_type, speed,
            ignore_all_collisions
        ]
        return self._CallMove(tokens)

    def OffroadToHub(self,
                     workspace,
                     hub,
                     opt,
                     timeout,
                     fallback_to_nominal=False,
                     project_name=None,
                     speed=0.1):
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
            speed (float): Speed factor of the move between 0.0 and 1.0

        Returns:
            (uint, uint): (return code - 0 means succes, sequence number - used for WaitForMove)
        """
        project_name = self._project_name if (project_name is None) else project_name
        tokens = [
            self.OFFROAD_TO_HUB, project_name, workspace, hub, opt, timeout, fallback_to_nominal,
            speed
        ]
        return self._CallMove(tokens)

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
        project_name = self._project_name if (project_name is None) else project_name
        tokens = [self.ACQUIRE_CONTROL, project_name, workspace]
        return self._CallCode(tokens)

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
        project_name = self._project_name if (project_name is None) else project_name
        tokens = [self.RELEASE_CONTROL, project_name, workspace]
        return self._CallCode(tokens)

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
        project_name = self._project_name if (project_name is None) else project_name
        tokens = [self.CANCEL_MOVE, project_name, workspace]
        return self._CallCode(tokens)

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
        project_name = self._project_name if (project_name is None) else project_name
        tokens = [self.CHANGE_WORKSPACE, project_name, workspace]
        return self._CallCode(tokens)

    def ChoreographyInstall(self, filepath):
        """
        Sends an ChoreographyInstall command.

        Parameters:
            filepath (string): path to choreography.json file, inside wb project

        Returns:
            unsigned int: return code 0 means success
        """
        tokens = [self.CHOREOGRAPHY_INSTALL, filepath]
        return self._CallCode(tokens)

    def ChoreographyUninstall(self, group_name):
        """
        Sends an ChoreographyUninstall command.

        Parameters:
            group_name (string): name of an installed choreography group

        Returns:
            unsigned int: return code 0 means success
        """
        tokens = [self.CHOREOGRAPHY_UNINSTALL, group_name]
        return self._CallCode(tokens)

    def ChoreographyLoad(self, choreography_name):
        """
        Sends an ChoreographyLoad command.

        Parameters:
            choreography_name (string): name of a choreography group installed in the appliance

        Returns:
            unsigned int: return code 0 means success
        """
        tokens = [self.CHOREOGRAPHY_LOAD, choreography_name]
        return self._CallCode(tokens)

    def ChoreographyUnload(self):
        """
        Sends an ChoreographyUnload command. Removes choreography from cache.

        Returns:
            unsigned int: return code 0 means success
        """
        tokens = [self.CHOREOGRAPHY_UNLOAD]
        return self._CallCode(tokens)

    def ChoreographyStage(self):
        """
        Sends an ChoreographyStage command. Instantiates and connects Appliance to Robots and gets
        ready to run the loaded choreography

        Returns:
            unsigned int: return code 0 means success
        """
        tokens = [self.CHOREOGRAPHY_STAGE]
        return self._CallCode(tokens)

    def ChoreographyUnstage(self):
        # TODO (swapnil) - this should be replaced with its own ASCII Command
        tokens = [self.CHOREOGRAPHY_UNSTAGE]
        return self._CallCode(tokens)

    def ChoreographyRun(self, auto_step_at_target):
        """
        Sends an ChoreographyRun command. Instantiates and connects Appliance to Robots and gets
        ready to run the loaded choreography

        Parameters:
            auto_step_at_target (bool): if true, robots will auto re-acquire after releasing at each
                target

        Returns:
            unsigned int: return code 0 means success
        """
        tokens = [self.CHOREOGRAPHY_RUN, 1 if auto_step_at_target else 0]
        code, data = self._SendAndRecvHelper(tokens, self._choreo_socket)
        return code

    def ChoreographyCancel(self):
        """
        Sends an ChoreographyCancel command. Stops any running choreography.

        Returns:
            unsigned int: return code 0 means success
        """
        tokens = [self.CHOREOGRAPHY_CANCEL]
        return self._CallCode(tokens)

    def ChoreographyStep(self, performer_name, segment_index, cycle_number):
        """
        Sends an ChoreographyStage command. Instantiates and connects Appliance to Robots and gets
        ready to run the loaded choreography

        Parameters:
            performer_name (str): name of the performer to step to next target
            segment_index (unsigned int): must match which segment the robot is currently waiting at

        Returns:
            unsigned int: return code 0 means success
        """
        tokens = [self.CHOREOGRAPHY_STEP, performer_name, segment_index, cycle_number]
        return self._CallCode(tokens)

    def ChoreographyWaitForResp(self, timeout):
        self._choreo_socket.settimeout(timeout)  # set the timeout
        try:
            data = self._choreo_socket.recv(PythonApplianceCommander.MAX_BUFFER_SIZE)  # try to recv
            print('received some data {}'.format(data))

        except Exception as e:
            print("Timed out waiting for delayed response, got {}".format(e))
            return None

        return self._SplitToTokens(data.decode('utf-8'))

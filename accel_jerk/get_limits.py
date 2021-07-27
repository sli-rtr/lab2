#!/usr/bin/env python3

# this program returns the experimental accel and jerk limits
# command line arguments get_limits(filename, ip, safety margin)

from __future__ import print_function
from __future__ import division

import sys
# sys.path.append('/home/samuelli/lab2/accel_jerk/rtr_control_ros/')
import rospy
from rtr_control_ros.srv import *
from rtr_control_ros.msg import *

import os, time
from datetime import datetime
import argparse
from concurrent.futures import ThreadPoolExecutor
import shutil
from pathlib import Path, PurePath
# import os.path
# from scipy.optimize import minimize
# import scipy.optimize
# from scipy.optimize import differential_evolution
import matplotlib.pyplot as plt
import numpy as np

from rtr_appliance.PythonApplianceCommander import PythonApplianceCommander
import update_limits
import compare_data
from ApiHelper import ApiHelper
import CommonOperations as cmn_ops

# from rtr_control_ros.log_plots import *
# from rtr_control_ros.robot_logs import RobotLogs

# def LaunchMoveToHub(cmdr,workstate,hub,speed,corner_smoothing,project):
#     # Execute this in a thread (MoveToHub function call)
#     res,seq = cmdr.MoveToHub(workstate,hub,speed,corner_smoothing,project_name=project)
#     code = cmdr.WaitForMove(seq)
#     return code


class limitTester():
    replan_attempts = 1
    timeout = 0.5

    def __init__(self, ip, fp):
        # Setup the PythonCommander which is responsible for sending commands to the
        # RTR Controller that control the robot/s or simulation
        self.initialized = False
        self.init_logging(fp)
        self.log('Replan Attempts: {}'.format(self.replan_attempts))
        self.log('Move Timeout: {}'.format(self.timeout))

        self.ip_addr = ip
        self.cmdr = PythonApplianceCommander(self.ip_addr, 9999)
        self.cmdr.Reconnect()

        code, data = self.cmdr.GetMode()
        if data == 'FAULT':
            code = self.cmdr.ClearFaults()
            if code != self.cmdr.SUCCESS:
                self.log('Failed to clear faults!')
                return

        # Commander helper that communicates with the controller using a REST api
        self.helper = ApiHelper(self.ip_addr)

        self.initialized = True

    def LaunchPickAndPlace(self, cmdr, workstate, hub, speed, corner_smoothing, project):
        # Execute this in a thread (Pick and Place motion sequence)
        res, seq = cmdr.MoveToHub(workstate, hub, speed, corner_smoothing, project_name=project)
        return cmdr.WaitForMove(seq)  # pick_code + place_code

    def init_logging(self, fp):
        self.fp = fp
        self.fp.write('\n')

    def log(self, msg):
        log_msg = '[{}] {}\n'.format(datetime.now(), msg)
        self.fp.write(log_msg)
        print(log_msg)

    def pick_and_place_part(self):
        project = self.project
        workstate = 'default_state'
        hub_list = self.hub_list
        hub_idx = self.hub_idxs

        if hub_idx > len(hub_list) - 1:
            self.end_time = time.time()
            self.unfinished = False
            self.thread = None
            self.log('Project {} has finished!'.format(self.project))
        else:
            hub = hub_list[hub_idx]
            future = self.executor.submit(self.LaunchPickAndPlace, self.cmdr, workstate, hub,
                                          self.speed, self.corner_smoothing, project)
            self.thread = future

    def retract_to_staging(self):
        # check and recover from faults
        if self.cmdr.GetMode()[1].strip() == 'FAULT':
            cmn_ops.attempt_fault_recovery(
                self.cmdr, self.project_info, self.group_name, hub=self.hub_list[0])

        self.log('Retracting project {} to staging!'.format(self.project))
        project = self.project
        workstate = 'default_state'
        hub = self.hub_list[0]
        future = self.executor.submit(self.LaunchPickAndPlace, self.cmdr, workstate, hub,
                                      self.speed, self.corner_smoothing, project)
        self.thread = future

    def add_group(self, group):
        '''
        This function adds the group to the rapidplan interface and makes sure it is unloaded.

        Parameters:
            group (string): name of group
        '''
        # Create a deconfliction group
        if group not in self.helper.get_installed_groups():
            self.log('Adding group')
            self.helper.post_create_group(self.group_name)

        # unload group if it is loaded already
        if self.helper.get_group_info()[self.group_name]['loaded'] == True:
            self.log('Unloading group')
            self.helper.put_unload_group(self.group_name)

    def add_project(self, proj, controller=1, ip='192.168.1.19'):
        '''
        This function loads a project, adds it to the group, and sets the parameters for controller type

        Parameters:
            proj (string): path to zip file '../somedir/project.zip'
            controller (int): use internal simulated 1 or robot controller 0
            ip (string): ip address of robot controller
        '''
        # should check if project exists already-remove if so??
        project_list = self.helper.get_installed_projects()
        self.project = os.path.splitext(os.path.basename(proj))[0]
        self.path_to_bin = os.path.join('/home/samuelli/rapidplan/install/var/log/rtr/robots/',
                                        self.project + '.bin')

        if self.project not in project_list:
            # upload project
            self.helper.post_install_project(proj)

            # wait until project added and fully loaded...
            while len(self.helper.get_installed_projects()) == len(project_list):
                time.sleep(1)
            time.sleep(2)

            # add project to group
            self.helper.put_project_to_group(self.group_name, self.project)

        else:
            self.log('Project already added, proceeding')

        # set parameters of project
        # set connection type: 0 for robot controller, 1 for simulated, 2 for third party sim
        self.helper.patch_robot_params(self.project, {'connection_type': controller})
        self.helper.patch_robot_params(
            self.project, {
                "robot_params": {
                    "bool_params": {
                        "using_urcap": False
                    },
                    "double_params": {},
                    "float_params": {
                        "max_tool_speed": 100
                    },
                    "int_params": {
                        "urcap_float_register": 0
                    },
                    "string_params": {
                        "robot_address": ip,
                        "sim_ip_address": "127.0.0.1"
                    },
                    "uint_params": {
                        "sim_command_port": 30003,
                        "sim_data_port": 30004
                    }
                }
            })

    # def cancel_move(self,workstate,project):
    #     self.cmdr.CancelMove('default_state',self.project)

    def load_run_through_hubs(self):
        '''
        This function loads the group, identifies the hubs and moves from the first to the last hub

        Parameters:
            proj (string): path to zip file '../somedir/project.zip'
            controller (int): use internal simulated 1 or robot controller 0
            ip (string): ip address of robot controller
        '''
        # get hubs
        self.group_info = self.helper.get_group_info()
        self.project_info = self.helper.get_project_info(
            self.group_info[self.group_name]['projects'])
        self.hubs = self.project_info[self.project]['hubs']
        all_hubs = []

        for hub in self.hubs:
            all_hubs.append(hub['name'])
        all_hubs.sort()
        # start from first and go directly to last
        self.hub_list = [all_hubs[0], all_hubs[-1]]
        print(self.hub_list)

        # Set interupt behavior
        self.cmdr.SetInterruptBehavior(
            self.replan_attempts, self.timeout, project_name=self.project)

        # Load deconfliction group
        self.helper.put_load_group(self.group_name)

        # Call startup sequence which calls InitGroup for each project and then BeginOperationMode
        self.log('Startup sequence...')
        resp = cmn_ops.startup_sequence(self.cmdr, self.project_info, self.group_name)
        if resp != 0:
            print('Startup sequence failed with error code: {}'.format(resp))
            return

        # Put each robot on the roadmap
        self.log('Putting robots on the roadmap...')
        move_res = cmn_ops.put_on_roadmap(
            self.cmdr, self.project_info, self.group_name, hub=self.hub_list[0])

        if move_res != None:
            if sum(move_res) != 0:
                self.log('Failed to put the robots on the roadmap')
                return

        # run through hubs
        self.executor = ThreadPoolExecutor(max_workers=1)
        self.thread = None
        self.unfinished = True
        self.end_time = None
        self.hub_idxs = 0
        self.interlocking = False

        self.log('Beginning hub move task...')
        self.start_time = time.time()

        self.pick_and_place_part()

        while self.unfinished:
            if self.thread.done():
                code = self.thread.result()

                if code == self.cmdr.SUCCESS:
                    if not self.interlocking:
                        # If your previous move was not a retraction, that means you completed a pick and place move
                        # and need to increase the hub idx
                        self.log('Project {} completed move to {}!'.format(
                            self.project, self.hub_list[self.hub_idxs]))
                        self.hub_idxs += 1
                    self.pick_and_place_part()
                    self.interlocking = False
                else:
                    # The requested move was blocked by the other robot, so retract to the staging position
                    self.retract_to_staging()
                    self.interlocking = True

        hub_time_str = '{} hub move task took: {}'.format(self.project,
                                                          self.end_time - self.start_time)
        self.log(hub_time_str)

    # def run_test(self,proj,controller,reference):
    #     return

    def save_bin(self, path, accel, jerk):
        '''
        TODO: remove hardcoded os path

        This function takes the automatically generated bin file in rapidplan/install/var/log/rtr/robots/[project].bin and
        copies it to another directory with the updated name [project]_[acceleration]_[jerk].bin

        Parameters:
            path (string): path to bin file
            accel (float): Tested acceleration value
            jerk (float): Tested jerk value
        '''
        # save bin BEFORE deleting project??
        # create/prompt for directory
        # bins located at ~/rapidplan/install/var/log/rtr/robots/[project].bin
        # do megadebs have bin files?
        aj_name = os.path.join(self.project + '_' + str(accel) + '_' + str(jerk) + '.bin')
        # check bin exists
        print(path)
        print(aj_name)
        if os.path.isfile(path):
            print("moving bin")
            # create dir

            # '/home/samuelli/rapidplan/install/var/log/rtr/robots/'
            Path('/home/samuelli/lab2/accel_jerk/bin_temp').mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, os.path.join('/home/samuelli/lab2/accel_jerk/bin_temp/', aj_name))
        else:
            self.log('Bin does not exist, skipping')

    def binary_search(self, proj, controller, reference, tester, accel, mse, val_found):
        '''
        This function searches for the global minima MSE and its corresponding acceleration

        Parameters:
            proj (string): path to zip file '../somedir/project.zip'
            controller (int): use internal simulated 1 or robot controller 0
            reference (string): path to reference trajectory bin file
            accel (float): acceleration value being tested
            mse (float): mse value at corresponding accels
            val_found (int[self.num_joints]): keeps track of whether the min value has been found for a joint
        '''
        midpoints = np.zeros((self.num_joints, 3))
        midpoint_mse = {}
        midpoint_set = set()

        if sum(val_found) == self.num_joints:
            return

        for i in range(self.num_joints):
            if val_found[i]:
                continue
            # how precise we need to be
            high = accel[i][4]
            low = accel[i][0]
            if high - low > 2:
                midpoints[i][1] = round((high + low) / 2)
                midpoints[i][0] = round((low + midpoints[i][1]) / 2)
                midpoints[i][2] = round((high + midpoints[i][1]) / 2)

                midpoint_set.update(midpoints[i])
            else:
                val_found[i] = 1
                print('joint', i, 'MSE done')
                print('accel', accel[i][0])
                # continue

        for i in midpoint_set:
            # skip 0 values, means we are within threshold
            if i == 0:
                continue

            if tester == 0:
                mse_mid = self.compute_rapidplan_mse(proj, controller, reference, i, 10000.0)
            else:
                mse_mid = self.compute_traj_server_mse(proj, controller, reference, i, 10000.0)

            # add to dict
            midpoint_mse[i] = mse_mid

        # print(midpoint_mse)

        for i in range(self.num_joints):
            # if joint done
            if val_found[i]:
                continue
            accel[i][1:4] = midpoints[i][0:3]
            # if midpoints[i] == 0: # bad fix... its because midpoint_mse = 0 does not exist in dict
            #     mse[i][1] = 0
            # else:
            mse[i][1:4] = [midpoint_mse[midpoints[i][j]][i] for j in range(0, 3)]

        # print(accel)
        # print(mse)

        # mse and accel_search_vals are known, we have a [low, mid, high] for each joint
        for i in range(self.num_joints):
            # if joint done
            if val_found[i]:
                continue

            # print(accel)
            # print(mse[i])
            a, b, c, d, e = mse[i]
            v, w, x, y, z = accel[i]

            # shrink both sides
            if a >= b >= c and e >= d >= c:
                a, e = b, d
                v, z = w, y
                # print("first")
            # search left side
            elif (a >= b < c and e >= d >= c) or (c > b or c > d and b < d):
                e, c = c, b
                z, x, = x, w
                # print('second')
            # search right side
            elif (a >= b >= c and e >= d < c) or (c > b or c > d and b >= d):
                a, c = c, d
                v, x = x, y
                # print('third')
            # catch local minima/non monotonic parts
            elif b > a:
                # print('fourth')
                a = b
                v = w
            elif d > e:
                # print('sixth')
                e = d
                z = y
            else:
                print('shrinking bounds failed with mse {}, accel {}'.format(mse[i], accel[i]))
            b, d = 0, 0
            w, y = 0, 0

            mse[i] = [a, b, c, d, e]
            accel[i] = [v, w, x, y, z]

        # print(accel)
        # print(mse)

        # plt.plot(accel[4], mse[4])
        # plt.show()

        self.binary_search(proj, controller, reference, tester, accel, mse, val_found)

    # for scipy minimize function to compute mse
    def compute_rapidplan_mse(self, proj, controller, reference, accel, jerk=5000.0):
        '''
        This function

        Parameters:
            proj (string): path to zip file '../somedir/project.zip'
            controller (int): use internal simulated 1 or robot controller 0
            reference (string): path to reference trajectory bin file
            accel (float): acceleration value being tested
            jerk (float): jerk value being tested
        '''
        self.log('Computing MSE for accel: {} jerk: {}'.format(accel, jerk))

        # returns mse value for single joint at a/j
        update_limits.parse(proj, accel, jerk)

        # add group
        self.add_group(self.group_name)
        # load project and add to group the first time
        self.add_project(proj, controller)
        # load group and run through hubs
        self.load_run_through_hubs()

        # end operation mode, put into config mode
        self.helper.put_config_mode()
        # unload group
        self.helper.put_unload_group(self.group_name)

        self.save_bin(self.path_to_bin, accel, jerk)

        # # test data here
        # # get measure of difference between commanded route and reference
        # # dataname = os.path.join('./bin_temp/'+'isolated_joint_move_Robot1'+'_'+str(accel_max)+'_'+str(5000.0)+'.bin')

        pos_r, vel_r, t_r = compare_data.get_data(reference)
        pos_d, vel_d, t_d = compare_data.get_data(self.path_to_bin)
        pos = [pos_r, pos_d]
        mse = compare_data.get_mse(pos_r, pos_d)
        # pos, vel, t = compare_data.get_data(reference, self.path_to_bin)
        # mse = compare_data.get_mse(pos)
        # print(mse)

        # remove project from deconf group
        self.helper.delete_proj_from_group(self.group_name, self.project)

        # unload project
        self.helper.delete_project(self.project)

        return mse

    def compute_traj_server_mse(self, proj, controller, reference, accel, jerk=10000.0):
        '''
        This function

        Parameters:
            proj (string): path to zip file '../somedir/project.zip'
            controller (int): use internal simulated 1 or robot controller 0
            reference (string): path to reference trajectory bin file
            accel (float): acceleration value being tested
            jerk (float): jerk value being tested
        '''
        self.log('Computing traj server MSE for accel: {} jerk: {}'.format(accel, jerk))
        rospy.wait_for_service('rtr_generate_traj_for_limits_estimation')
        try:
            traj_gen = rospy.ServiceProxy('rtr_generate_traj_for_limits_estimation',
                                          GenTrajForLimitsEstimate)
            traj_gen_request = GenTrajForLimitsEstimateRequest()
            traj_gen_request.robot_type = "UR5"
            traj_gen_request.sample_period = 0.008  # set sample period to match avg bin file 'sampling' period
            traj_gen_request.acc_limits.vec = [accel] * 6
            traj_gen_request.jerk_limits.vec = [jerk] * 6

            a = DoubleVec()
            a.vec = [0, 0, 0, 0, 0, 0]
            aa = DoubleVec()
            aa.vec = [0, 0, 0, 0, 0, 2 * np.pi]
            b = DoubleVec()
            b.vec = [0, 0, 0, 0, 0, -2 * np.pi]
            c = DoubleVec()
            c.vec = [0, 0, 0, 0, 2 * np.pi, -2 * np.pi]
            d = DoubleVec()
            d.vec = [0, 0, 0, 0, -2 * np.pi, -2 * np.pi]
            e = DoubleVec()
            e.vec = [0, 0, 0, 2 * np.pi, -2 * np.pi, -2 * np.pi]
            f = DoubleVec()
            f.vec = [0, 0, 0, -2 * np.pi, -2 * np.pi, -2 * np.pi]
            g = DoubleVec()
            g.vec = [0, 0, 125.0 / 180.0 * np.pi, -2 * np.pi, -2 * np.pi, -2 * np.pi]
            h = DoubleVec()
            h.vec = [0, 0, -160.0 / 180.0 * np.pi, -2 * np.pi, -2 * np.pi, -2 * np.pi]
            i = DoubleVec()
            i.vec = [0, 0, 0, -2 * np.pi, -2 * np.pi, -2 * np.pi]
            j = DoubleVec()
            j.vec = [0, 35.0 / 180.0 * np.pi, 0, -2 * np.pi, -2 * np.pi, -2 * np.pi]
            k = DoubleVec()
            k.vec = [0, -185.0 / 180.0 * np.pi, 0, -2 * np.pi, -2 * np.pi, -2 * np.pi]
            l = DoubleVec()
            l.vec = [0, -np.pi, 0, -2 * np.pi, -2 * np.pi, -2 * np.pi]
            m = DoubleVec()
            m.vec = [45.0 / 180.0 * np.pi, -np.pi, 0, -2 * np.pi, -2 * np.pi, -2 * np.pi]
            n = DoubleVec()
            n.vec = [-215.0 / 180.0 * np.pi, -np.pi, 0, -2 * np.pi, -2 * np.pi, -2 * np.pi]
            o = DoubleVec()
            o.vec = [0, 0, 0, 0, -2 * np.pi, -2 * np.pi]
            p = DoubleVec()
            p.vec = [0, 0, 0, 0, 0, 0]
            traj_gen_request.waypoints = [a, aa, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p]

            resp = traj_gen(traj_gen_request)

            # resp.velocity is doublevec object. need to create array, and transpose the shape
            # print(resp.velocity[0].vec[0])
            # pos_d = np.transpose(np.array(resp.velocity))
            pos_c = [[resp.samples[i].vec[j]
                      for j in range(len(resp.samples[0].vec))]
                     for i in range(len(resp.samples))]
            vel_c = [[resp.velocity[i].vec[j]
                      for j in range(len(resp.velocity[0].vec))]
                     for i in range(len(resp.velocity))]
            time_c = np.arange(0, traj_gen_request.sample_period * len(pos_c),
                               traj_gen_request.sample_period)
            pos_c = np.transpose(pos_c)

            start, end = compare_data.get_end_times(vel_c, 0.1)
            # vel_c = np.transpose(vel_c)
            p_c_trim, v_c_trim = [], []
            t_trim = []

            # extract only interesting data
            for i in range(len(pos_c)):
                p_c_trim.append(pos_c[i][start[i]:end[i]])
                # v_c_trim.append(vel_c[i][start[i]:end[i]])
                t_trim.append(
                    [j - time_c[start[i]] for j in time_c[start[i]:end[i]]])  # normalize times

            pos_r, vel_r, t_r = compare_data.get_data(reference)
            pos = [pos_r, p_c_trim]
            time = [t_r, t_trim]
            mse = compare_data.get_mse(pos[0], pos[1])
            # print(mse)

            # compare_data.plot_data(time, pos, mse)
            # plt.show()

            return mse
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def start(self, proj, controller, reference, tester):
        '''
        parameters
        proj: path to zip file '../somedir/project.zip'
        controller: use internal simulated 1 or robot controller 0
        reference: reference bin file of teach pendant control
        tester:
        '''
        #Check if the initialization was successful and end the program if initialization failed
        if self.initialized == False:
            return

        # iterate through accel/jerk values
        accel_test = 16.0
        jerk_test = 16.0
        jerk_max = 10000.0

        # set project urdf
        update_limits.parse(proj, accel_test, jerk_max)

        self.speed = 0.99
        self.corner_smoothing = 0.0

        opt_time_start = time.time()

        # set flag for testing accel or jerk
        self.test_accel = True

        ### this will run and plot
        # a = np.linspace(1, 16, 16)
        # mse_vec = np.vectorize(self.compute_mse)
        # y = mse_vec(a, proj, controller, reference, 5, jerk_max)
        # plt.plot(a, y)
        # plt.show()

        ### this will plot existing data
        # a = np.linspace(16, 64, 49)
        # mse = []
        #
        # for i in range(16,65):
        #     dataname = os.path.join('./bin_temp/'+'isolated_joint_move_Robot1'+'_'+str(i)+'.0_'+str(5000.0)+'.bin')
        #     pos, vel, t = compare_data.get_data(reference, dataname)
        #     mse.append(compare_data.get_mse(pos))
        #     print('running')
        #
        # for i in range(6):
        #     plt.subplot(6, 1, i+1)
        #     data = [j[i] for j in mse]
        #     plt.title('joint {}'.format(i))
        #     plt.plot(a, data)
        #
        #
        # plt.show()

        mse_prev = []
        mse_delta = []
        mse_search_vals = []

        aj_search_vals = []

        max_a_found = []
        max_j_found = []
        max_found = []

        self.group_name = 'accel_jerk_test'

        # run twice, once for accel, again for jerk
        for i in range(2):
            # searches for upper bound a/j values
            print('run', i)
            while True:
                if tester == 0:
                    mse = self.compute_rapidplan_mse(proj, controller, reference, accel_test, jerk_max)
                else:
                    mse = self.compute_traj_server_mse(proj, controller, reference, accel_test, jerk_max)

                self.num_joints = len(mse)

                # test the mse data
                # init on first run
                if (self.test_accel == True and len(max_a_found) == 0) or (self.test_accel == False and len(max_j_found) == 0):
                    print("resetting values")
                    mse_prev = (np.ones(self.num_joints) * np.inf)
                    mse_delta = np.zeros(self.num_joints)
                    mse_search_vals = np.zeros((self.num_joints, 5))

                    aj_search_vals = np.zeros((self.num_joints, 5))

                    if self.test_accel == True and len(max_a_found) == 0:
                        print('reset accel')
                        max_a_found = np.zeros(self.num_joints)
                        max_found = max_a_found
                    else:
                        print('reset jerk')
                        max_j_found = np.zeros(self.num_joints)
                        max_found = max_j_found

                for j in range(self.num_joints):
                    # store mse vals associated with accel
                    if max_found[j] == 0:
                        # 2 is unnecessary, can be temp
                        mse_search_vals[j][0] = mse_search_vals[j][2]
                        mse_search_vals[j][2] = mse_search_vals[j][4]
                        mse_search_vals[j][4] = mse[j]

                    mse_delta[j] = mse[j] - mse_prev[j]
                    if mse_delta[j] < 0:
                        # new mse is less than the previous mse
                        continue
                    else:
                        # new mse is worse than the previous mse`
                        # add value to aj_search_vals the first time
                        # mse threshold? change magic number
                        if aj_search_vals[j][4] == 0 and mse[j] < 1:
                            if self.test_accel == True:
                                aj_search_vals[j] = [accel_test / 4, 0, 0, 0, accel_test]
                            else:
                                aj_search_vals[j] = [jerk_max / 4, 0, 0, 0, jerk_max]
                            max_found[j] = 1


                if self.test_accel == True:
                    max_a_found = max_found
                else:
                    max_j_found = max_found


                mse_prev = mse

                # found max for all joints
                if self.test_accel == True and sum(max_a_found) >= self.num_joints:
                    # self.test_accel == False
                    print('found accel max')
                    break
                elif self.test_accel == False and sum(max_j_found) >= self.num_joints:
                    print('found jerk max')
                    break

                if self.test_accel == True:
                    accel_test *= 2
                else:
                    jerk_test *= 2
                    jerk_max = jerk_test

                # edit project urdf
                update_limits.parse(proj, accel_test, jerk_max)

            # searches between upper and lower bound
            val_found = np.zeros(self.num_joints)
            self.binary_search(proj, controller, reference, tester, aj_search_vals, mse_search_vals,
                               val_found)

            self.test_accel = False

        # return determined values

        # clean up stuff
        # delete deconfliction group
        if tester == 0:
            self.helper.delete_group(self.group_name)

        opt_time_str = 'optimization task took: {} seconds'.format(time.time() - opt_time_start)
        self.log(opt_time_str)

        return

def main():
    parser = argparse.ArgumentParser(description='Get joint limits for accel and jerk')
    parser.add_argument(
        "-f",
        "--file",
        help="File path to project file. Required argument.",
        type=str,
        required=True)
    parser.add_argument(
        "-r",
        "--reference",
        help="File path to reference control bin file. Required argument.",
        type=str,
        required=True)
    parser.add_argument(
        "-i",
        "--ip",
        help="ip address of realtime controller. Default 127.0.0.1.",
        type=str,
        default="127.0.0.1",
        required=False)
    parser.add_argument(
        "-c",
        "--controller",
        help="Using robot controller [0] or internal simulated [1]. Default internal controller.",
        type=int,
        default=1,
        required=False)
    parser.add_argument(
        "-t",
        "--tester",
        help=
        "Using rapidplan controller [0] or trajectory generation server [1]. Default rapidplan controller.",
        type=int,
        default=0,
        required=False)

    args = parser.parse_args()
    fname = args.file
    reference = args.reference
    ip_addr = args.ip
    controller = args.controller
    tester = args.tester

    # # If ip address is passed, use it
    # if len(sys.argv) != 1:
    #     ip_addr = str(sys.argv[1])
    #     fname = str(sys.argv[2])
    # else: # Default IP address
    #     ip_addr = "127.0.0.1"
    # print(f'Setting ip address of Realtime Controller to: {ip_addr}')

    fp = open('hub_log.txt', 'a')
    try:
        task_planner = limitTester(ip_addr, fp)
        task_planner.start(fname, controller, reference, tester)
    finally:
        fp.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print('KeyboardInterrupt, program halted')
        try:
            sys.exit(1)
        except SystemExit:
            os._exit(1)

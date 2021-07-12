#!/usr/bin/env python3

# this program returns the experimental accel and jerk limits
# command line arguments get_limits(filename, ip, safety margin)

import sys, os, time
from datetime import datetime
import argparse
from concurrent.futures import ThreadPoolExecutor
import shutil
from pathlib import Path, PurePath

from rtr_appliance.PythonApplianceCommander import PythonApplianceCommander
import update_limits
import compare_data
from ApiHelper import ApiHelper
import CommonOperations as cmn_ops

from rtr_control_ros.log_plots import *
from rtr_control_ros.robot_logs import RobotLogs

def LaunchMoveToHub(cmdr,workstate,hub,speed,corner_smoothing,project):
    # Execute this in a thread (MoveToHub function call)
    res,seq = cmdr.MoveToHub(workstate,hub,speed,corner_smoothing,project_name=project)
    code = cmdr.WaitForMove(seq)
    return code

class limitTester():
    replan_attempts = 1
    timeout = 0.5

    def __init__(self,ip,fp):
        # Setup the PythonCommander which is responsible for sending commands to the
        # RTR Controller that control the robot/s or simulation
        self.initialized = False
        self.init_logging(fp)
        self.log(f'Replan Attempts: {self.replan_attempts}')
        self.log(f'Move Timeout: {self.timeout}')

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

    def LaunchPickAndPlace(self, cmdr,workstate, hub, speed,corner_smoothing,project):
        # Execute this in a thread (Pick and Place motion sequence)
        res,seq = cmdr.MoveToHub(workstate,hub,speed,corner_smoothing,project_name=project)
        return cmdr.WaitForMove(seq)   # pick_code + place_code

    def init_logging(self,fp):
        self.fp = fp
        self.fp.write('\n')

    def log(self,msg):
        log_msg = f'[{datetime.now()}] {msg}\n'
        self.fp.write(log_msg)
        print(log_msg)

    def pick_and_place_part(self):
        project = self.project
        workstate = 'default_state'
        hub_list = self.hub_list
        hub_idx = self.hub_idxs

        if hub_idx > len(hub_list)-1:
            self.end_time = time.time()
            self.unfinished = False
            self.thread = None
            self.log(f'Project {self.project} has finished!')
        else:
            hub = hub_list[hub_idx]
            future = self.executor.submit(self.LaunchPickAndPlace,self.cmdr,workstate,hub, self.speed,self.corner_smoothing,project)
            self.thread = future

    def retract_to_staging(self):
        # check and recover from faults
        if self.cmdr.GetMode()[1].strip() == 'FAULT':
            cmn_ops.attempt_fault_recovery(self.cmdr,self.project_info,self.group_name,hub=self.hub_list[0])

        self.log(f'Retracting project {self.project} to staging!')
        project = self.project
        workstate = 'default_state'
        hub = self.hub_list[0]
        future = self.executor.submit(LaunchMoveToHub,self.cmdr,workstate,hub,self.speed,self.corner_smoothing,project)
        self.thread = future

    def add_group(self,group):
        '''
        parameters
        group: name of group
        '''
        self.group_name = group

        # Create a deconfliction group
        if self.group_name not in self.helper.get_installed_groups():
            self.log('Adding group')
            self.group = self.helper.post_create_group(self.group_name)

        # unload group if it is loaded already
        if self.helper.get_group_info()[self.group_name]['loaded'] == True:
            self.log('Unloading group')
            self.helper.put_unload_group(self.group_name)

    def add_project(self,proj,controller):
        '''
        parameters
        proj: path to zip file '../somedir/project.zip'
        controller: use internal simulated 1 or robot controller 0
        '''
        # should check if project exists already-remove if so??
        project_list = self.helper.get_installed_projects()
        self.project = os.path.splitext(os.path.basename(proj))[0]

        if self.project not in project_list:
            # upload project
            self.helper.post_install_project(proj)

            # wait until project added and fully loaded... hacky shit
            while len(self.helper.get_installed_projects()) == len(project_list):
                time.sleep(1)
            time.sleep(2)

            # add project to group
            self.helper.put_project_to_group(self.group_name, self.project)

        else:
            self.log('Project already added, proceeding')

        # set parameters of project
        if controller == 0:
        # set connection type: 0 for robot controller, 1 for simulated, 2 for third party sim
            self.helper.patch_robot_params(self.project, {'connection_type':0})
            self.helper.patch_robot_params(self.project, {"robot_params":{
                "bool_params":{"using_urcap":False},
                "double_params":{},
                "float_params":{"max_tool_speed":100},
                "int_params":{"urcap_float_register":0},
                "string_params":{"robot_address":"192.168.1.19","sim_ip_address":"127.0.0.1"},
                "uint_params":{"sim_command_port":30003,"sim_data_port":30004}
                }})
        else:
            self.helper.patch_robot_params(self.project, {'connection_type':1}) # internal simulated
            self.helper.patch_robot_params(self.project, {"robot_params":{
                "bool_params":{"using_urcap":False},
                "double_params":{},
                "float_params":{"max_tool_speed":100},
                "int_params":{"urcap_float_register":0},
                "string_params":{"robot_address":"192.168.1.19","sim_ip_address":"127.0.0.1"},
                "uint_params":{"sim_command_port":30003,"sim_data_port":30004}
                }})

    # def cancel_move(self,workstate,project):
    #     self.cmdr.CancelMove('default_state',self.project)

    def load_run_through_hubs(self):
        # get hubs
        self.group_info = self.helper.get_group_info()
        self.project_info = self.helper.get_project_info(self.group_info[self.group_name]['projects'])
        self.hubs = self.project_info[self.project]['hubs']
        all_hubs = []

        for hub in self.hubs:
            all_hubs.append(hub['name'])
        all_hubs.sort()
        # start from first and go directly to last
        self.hub_list = [all_hubs[0], all_hubs[-1]]
        print(self.hub_list)

        # Set interupt behavior
        self.cmdr.SetInterruptBehavior(self.replan_attempts,self.timeout,project_name=self.project)

        # Load deconfliction group
        self.helper.put_load_group(self.group_name)

        # Call startup sequence which calls InitGroup for each project and then BeginOperationMode
        self.log('Startup sequence...')
        resp = cmn_ops.startup_sequence(self.cmdr,self.project_info,self.group_name)
        if resp != 0:
            print(f'Startup sequence failed with error code: {resp}')
            return

        # Put each robot on the roadmap
        self.log('Putting robots on the roadmap...')
        move_res = cmn_ops.put_on_roadmap(self.cmdr,self.project_info,self.group_name,hub=self.hub_list[0])

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

        self.log(f'Beginning hub move task...')
        self.start_time = time.time()

        self.pick_and_place_part()

        while self.unfinished:
            if self.thread.done():
                code = self.thread.result()

                if code == self.cmdr.SUCCESS:
                    if not self.interlocking:
                        # If your previous move was not a retraction, that means you completed a pick and place move
                        # and need to increase the hub idx
                        self.log(f'Project {self.project} completed move to {self.hub_list[self.hub_idxs]}!')
                        self.hub_idxs += 1
                    self.pick_and_place_part()
                    self.interlocking = False
                else:
                    # The requested move was blocked by the other robot, so retract to the staging position
                    self.retract_to_staging()
                    self.interlocking = True

        hub_time_str = f'{self.project} hub move task took: {self.end_time-self.start_time}'
        self.log(hub_time_str)

    def run_test(self,proj,controller,reference):
        return

    def save_bin(self, accel, jerk):
        # save bin BEFORE deleting project??
        # create/prompt for directory
        # bins located at ~/rapidplan/install/var/log/rtr/robots/[project].bin
        # do megadebs have bin files?
        self.path_to_bin = os.path.join('/home/samuelli/rapidplan/install/var/log/rtr/robots/', self.project + '.bin')
        aj_name = os.path.join(self.project+'_'+str(accel)+'_'+str(jerk)+'.bin')
        # check bin exists
        print(self.path_to_bin)
        print(aj_name)
        if os.path.isfile(self.path_to_bin):
            print("moving bin")
            # create dir
            Path('./bin_temp').mkdir(parents=True, exist_ok=True)
            shutil.copy2(self.path_to_bin, os.path.join('./bin_temp/', aj_name))
        else:
            self.log('Bin does not exist, skipping')

    def binary_search(self, proj, controller, reference, accel, mse, val_found):
        midpoints = np.zeros(self.num_joints)
        midpoint_mse = {}

        if sum(val_found) == self.num_joints:
            return

        for i in range(self.num_joints):
            if val_found[i]:
                continue
            # how precise we need to be
            if accel[i][2]-accel[i][0] > 2:
                # midpoint = (accel[i][2]+accel[i][0])/2
                midpoints[i] = (accel[i][2]+accel[i][0])/2
            else:
                val_found[i] = 1
                # midpoints[i] = 0
                # return lower? because mid is cleared to 0
                print('joint',i,'MSE done')
                print('accel',accel[i][0])
                # continue

        midpoint_set = set(midpoints)

        for i in midpoint_set:
            # skip 0 values, means we are within threshold
            if i == 0:
                continue

            update_limits.parse(proj, i, 5000.0)

            # add group
            self.add_group('accel_jerk_test')
            # load project and add to group the first time
            self.add_project(proj, controller)
            # load group and run through hubs
            self.load_run_through_hubs()

            # end operation mode, put into config mode
            self.helper.put_config_mode()
            # unload group
            self.helper.put_unload_group(self.group_name)

            self.save_bin(i, 5000.0)

            # # test data here
            # # get measure of difference between commanded route and reference

            pos, vel, t = compare_data.get_data(reference, self.path_to_bin)
            mse_mid = compare_data.get_mse(pos)
            print(mse_mid)

            # remove project from deconf group
            self.helper.delete_proj_from_group(self.group_name, self.project)

            # unload project
            self.helper.delete_project(self.project)

            # add to dict
            midpoint_mse[i] = mse_mid

        print(midpoint_mse)

        for i in range(self.num_joints):
            if val_found[i]:
                continue
            # # if joint done
            # if accel[i][1] == 0:
            #     continue
            print(accel)
            print(midpoints)
            print(mse)
            print(midpoint_mse)
            accel[i][1] = midpoints[i]
            # if midpoints[i] == 0: # bad fix... its because midpoint_mse = 0 does not exist in dict
            #     mse[i][1] = 0
            # else:
            mse[i][1] = midpoint_mse[midpoints[i]][i]

        print(accel)
        print(mse)

        # mse and accel_search_vals are known, we have a [low, mid, high] for each joint
        for i in range(self.num_joints):
            if val_found[i]:
                continue
            # # if joint done
            # if accel[i][1] == 0:
            #     continue
            if mse[i][0] < mse[i][2]:
                mse[i][2] = mse[i][1]
                accel[i][2] = accel[i][1]
                # print('searching', accel_search_vals[i][0], accel_search_vals[i][1])
                # binary_search(accel_search_vals[i][0], accel_search_vals[i][1])
            else:
                mse[i][0] = mse[i][1]
                accel[i][0] = accel[i][1]
                # print('searching', accel_search_vals[i][1], accel_search_vals[i][2])
                # binary_search(accel_search_vals[i][1], accel_search_vals[i][2])

            mse[i][1] = 0
            accel[i][1] = 0

        print(accel)
        print(mse)

        self.binary_search(proj, controller, reference, accel, mse, val_found)

        # return

    def start(self,proj,controller,reference):
        '''
        parameters
        proj: path to zip file '../somedir/project.zip'
        controller: use internal simulated 1 or robot controller 0
        reference: reference bin file of teach pendant control
        '''
        #Check if the initialization was successful and end the program if initialization failed
        if self.initialized == False:
            return

        # this function loops
        # iterate through accel/jerk values
        accel_min = 1.0
        accel_max = 16.0
        jerk_min = 10.0
        jerk_max = 5000.0
        # self.accel = [2.0, 2.0] # [low, high]
        # self.jerk = [2.0, 2.0]
        # set project urdf
        update_limits.parse(proj, accel_max, jerk_max)

        self.speed = 0.99
        self.corner_smoothing = 0.0

        mse_prev = []
        mse_delta = []

        accel_search_vals = []
        mse_search_vals = []

        max_found = []

        while True:
            # add group
            self.add_group('accel_jerk_test')
            # load project and add to group the first time
            self.add_project(proj, controller)
            # load group and run through hubs
            self.load_run_through_hubs()

            # end operation mode, put into config mode
            self.helper.put_config_mode()
            # unload group
            self.helper.put_unload_group(self.group_name)

            self.save_bin(accel_max, jerk_max)

            # # test data here
            # # get measure of difference between commanded route and reference

            pos, vel, t = compare_data.get_data(reference, self.path_to_bin)
            mse = compare_data.get_mse(pos)
            self.num_joints = len(pos[0])
            print(mse)

            # remove project from deconf group
            self.helper.delete_proj_from_group(self.group_name, self.project)

            # unload project
            self.helper.delete_project(self.project)

            print("-------------------------------\n\n\n\nSUCCEEDED\n\n\n\n-------------------------")
            print(accel_max)
            print(jerk_max)

            # test the mse data
            # init on first run
            if len(mse_prev) == 0:
                mse_prev = (np.ones(self.num_joints)*np.inf)
                mse_delta = np.zeros(self.num_joints)

            # if len(accel_search_vals) == 0:
                accel_search_vals = np.zeros((self.num_joints,3))
                mse_search_vals = np.zeros((self.num_joints,3))

                max_found = np.zeros(self.num_joints)

            for i in range(self.num_joints):
                # store mse vals associated with accel
                if max_found[i] == 0:
                    mse_search_vals[i][:2] = mse_search_vals[i][1:]
                    mse_search_vals[i][2] = mse[i]
                    # print(mse_search_vals)

                mse_delta[i] = mse[i]-mse_prev[i]
                if mse_delta[i] < 0:
                    continue
                    # print('joint',i,'good')
                    #good, continue
                else:
                    # print('joint',i,'getting worse')
                    # add value to accel_search_vals the first time
                    # mse threshold? change magic number
                    if accel_search_vals[i][2] == 0 and mse[i] < 1:
                        accel_search_vals[i] = [accel_max/8, accel_max/4, accel_max/2]
                        max_found[i] = 1

                    # print(accel_search_vals)
                    # print(mse_search_vals)

            mse_prev = mse

            if sum(max_found)>=self.num_joints:
                break

            accel_min = accel_max
            jerk_min = jerk_max
            accel_max *= 2
            # jerk_max *= 2

            # if run successful
                # if high-low value reach singularity
                    # self.done = true
                # else
                    # set low limit = high
                    # double high limit
            # else
                # set high to midpoint

            # edit project urdf
            update_limits.parse(proj, accel_max, jerk_max)

            # break



        # mse and accel_search_vals are known, we have a [low, mid, high] for each joint
        for i in range(self.num_joints):
            if mse_search_vals[i][0] < mse_search_vals[i][2]:
                mse_search_vals[i][2] = mse_search_vals[i][1]
                accel_search_vals[i][2] = accel_search_vals[i][1]
                # print('searching', accel_search_vals[i][0], accel_search_vals[i][1])
                # binary_search(accel_search_vals[i][0], accel_search_vals[i][1])
            else:
                mse_search_vals[i][0] = mse_search_vals[i][1]
                accel_search_vals[i][0] = accel_search_vals[i][1]
                # print('searching', accel_search_vals[i][1], accel_search_vals[i][2])
                # binary_search(accel_search_vals[i][1], accel_search_vals[i][2])

            mse_search_vals[i][1] = 0
            accel_search_vals[i][1] = 0

        print(accel_search_vals)
        print(mse_search_vals)

        val_found = np.zeros(self.num_joints)
        self.binary_search(proj, controller, reference, accel_search_vals, mse_search_vals, val_found)



        # return determined values


        # clean up stuff
        # delete deconfliction group
        self.helper.delete_group(self.group_name)


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

    args = parser.parse_args()
    fname = args.file
    reference = args.reference
    ip_addr = args.ip
    controller = args.controller

    # # If ip address is passed, use it
    # if len(sys.argv) != 1:
    #     ip_addr = str(sys.argv[1])
    #     fname = str(sys.argv[2])
    # else: # Default IP address
    #     ip_addr = "127.0.0.1"
    # print(f'Setting ip address of Realtime Controller to: {ip_addr}')

    fp = open('hub_log.txt','a')
    try:
        task_planner = limitTester(ip_addr,fp)
        task_planner.start(fname,controller,reference)
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

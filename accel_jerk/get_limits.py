#!/usr/bin/env python3

# this program returns the experimental accel and jerk limits

# this program should:
# - load toolkit URDF
# - edit joint limits for acceleration and jerk
# - zip and upload to Rapidplan
# - set robot IP/hostname
# - set speed 100%
# - run through path, Offroad to A, then to P
# - export logs
# - if succeeded, double joint limits
# - analysis stuff

# command line arguments get_limits(filename, ip, safety margin)

# import requests
import sys, os, time
from datetime import datetime
import argparse
from datetime import datetime
from concurrent.futures import ThreadPoolExecutor

from PythonApplianceCommander import PythonApplianceCommander
import update_limits
from ApiHelper import ApiHelper
import CommonOperations as cmn_ops

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

        # should check if group exists already?? remove if so?
        # group_list = self.helper.get_installed_groups()
        # if group_name not in group_list:
        #     print('install')
        #     # upload project
        #     self.project = self.helper.post_install_project(proj)
        # else:
        #     self.project = project_name
        # Create a deconfliction group
        self.group_name = 'temp_group_name'
        self.group = self.helper.post_create_group(self.group_name)


        # run test







        # # Request the group info from the group currently loaded on the control panel
        # self.group_info = self.helper.get_group_info()
        # self.group = None
        # for group_name,info in self.group_info.items():
        #     if info['loaded']:
        #         self.group = group_name
        #         self.project_names = info['projects']
        #
        # # Nested dictionary that contains all projects information of the form:
        # # {project name: {workstates: [str], hubs: [str]}
        # self.project_info = self.helper.get_project_info(self.group_info[self.group]['projects'])
        #
        # # Call startup sequence which calls InitGroup for each project and then BeginOperationMode
        # self.log('Startup sequence...')
        # resp = cmn_ops.startup_sequence(self.cmdr,self.project_info,self.group)
        # if resp != 0:
        #     print(f'Startup sequence failed with error code: {resp}')
        #     return
        #
        # # Put each robot on the roadmap
        # self.log('Putting robots on the roadmap...')
        # move_res = cmn_ops.put_on_roadmap(self.cmdr,self.project_info,self.group,hub='staging')
        #
        # if move_res != None:
        #     if sum(move_res) != 0:
        #         self.log('Failed to put the robots on the roadmap')
        #         return
        #
        # # Set interupt behavior
        # for project_idx in range(0,len(self.project_info)):
        #     name = self.project_names[project_idx]
        #     self.cmdr.SetInterruptBehavior(self.replan_attempts,self.timeout,project_name=name)

        self.initialized = True

    def LaunchPickAndPlace(self, cmdr,workstate, hub, speed,corner_smoothing,project):
        # Execute this in a thread (Pick and Place motion sequence)
        res,seq = cmdr.MoveToHub(workstate,hub,speed,corner_smoothing,project_name=project)
        # place_code = cmdr.WaitForMove(seq)
        #
        # if place_code != 0:
        #     place_code = 1
        # else:
        #     place_code = 0

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
        self.log(f'Retracting project {self.project} to staging!')
        project = self.project
        workstate = 'default_state'
        hub = self.hub_list[0]
        future = self.executor.submit(LaunchMoveToHub,self.cmdr,workstate,hub,self.speed,self.corner_smoothing,project)
        self.thread = future

    def binary_search(self):
        return

    def add_project(self,proj):
        '''
        parameters
        proj: path to zip file '../somedir/project.zip'
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
        self.helper.put_project_to_group(self.group, self.project)

        # # Load deconfliction group
        # self.helper.put_load_group(self.group)

    def start(self,proj):
        '''
        parameters
        proj: path to zip file '../somedir/project.zip'
        '''
        #Check if the initialization was successful and end the program if initialization failed
        if self.initialized == False:
            return

        # this function loops
        # iterate through accel/jerk values
        self.accel = [2.0, 2.0] # [low, high]
        self.jerk = [2.0, 2.0]

        self.speed = 0.8
        self.corner_smoothing = 0.0


        # starting_hub = self.hubs.

        # print(self.hubs)

        while True:
            # load project and add to group the first time
            self.add_project(proj)

            # get hubs
            self.group_info = self.helper.get_group_info()
            self.project_info = self.helper.get_project_info(self.group_info[self.group]['projects'])
            self.hubs = self.project_info[self.project]['hubs']
            self.hub_list = []

            for hub in self.hubs:
                self.hub_list.append(hub['name'])
            self.hub_list.sort()
            print(self.hub_list)

            # Set interupt behavior
            self.cmdr.SetInterruptBehavior(self.replan_attempts,self.timeout,project_name=self.project)

            # set connection type: 0 for robot controller, 1 for simulated, 2 for third party sim
            self.helper.patch_robot_params(self.project, {'connection_type':0})
            # self.helper.patch_robot_params(self.project, {'robot_params':{'string_params':{'robot_address':'192.168.1.19'}}})
            self.helper.patch_robot_params(self.project, {"robot_params":{
                "bool_params":{"using_urcap":False},
                "double_params":{},
                "float_params":{"max_tool_speed":5},
                "int_params":{"urcap_float_register":0},
                "string_params":{"robot_address":"192.168.1.19","sim_ip_address":"127.0.0.1"},
                "uint_params":{"sim_command_port":30003,"sim_data_port":30004}
                }})
            # self.helper.patch_robot_params(self.project, {'connection_type':1}) # internal simulated

            # Load deconfliction group
            self.helper.put_load_group(self.group)

            # Call startup sequence which calls InitGroup for each project and then BeginOperationMode
            self.log('Startup sequence...')
            resp = cmn_ops.startup_sequence(self.cmdr,self.project_info,self.group)
            if resp != 0:
                print(f'Startup sequence failed with error code: {resp}')
                return

            # Put each robot on the roadmap
            self.log('Putting robots on the roadmap...')
            move_res = cmn_ops.put_on_roadmap(self.cmdr,self.project_info,self.group,hub=self.hub_list[0])

            if move_res != None:
                if sum(move_res) != 0:
                    self.log('Failed to put the robots on the roadmap')
                    return



            # self.executor = ThreadPoolExecutor(max_workers=4)

            # run through hubs
            # alphabetical order?
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


            # end operation mode, put into config mode
            self.helper.put_config_mode()

            # unload group
            self.helper.put_unload_group(self.group_name)

            # remove project from deconf group
            self.helper.delete_proj_from_group(self.group_name, self.project)

            # unload project
            self.helper.delete_project(self.project)

            # save bin





            # if run successful
                # if high-low value reach singularity
                    # self.done = true
                # else
                    # set low limit = high
                    # double high limit
            # else
                # set high to midpoint

            # edit project urdf


            break

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
        "-i",
        "--ip",
        help="ip address of realtime controller. Default 127.0.0.1.",
        type=str,
        default="127.0.0.1",
        required=False)

    args = parser.parse_args()
    fname = args.file
    ip_addr = args.ip

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
        task_planner.start(fname)
    finally:
        fp.close()



    # update_limits('')

if __name__ == "__main__":
    main()

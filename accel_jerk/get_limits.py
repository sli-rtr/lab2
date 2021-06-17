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
import sys
from datetime import datetime

from PythonApplianceCommander import PythonApplianceCommander
import update_limits
from ApiHelper import ApiHelper

def LaunchMoveToHub(cmdr,workstate,hub,speed,project):
    # Execute this in a thread (MoveToHub function call)
    res,seq = cmdr.MoveToHub(workstate,hub,speed,project_name=project)
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
        # Create a deconfliction group
        self.group = self.helper.post_create_group('temp_group_name')


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

    def init_logging(self,fp):
        self.fp = fp
        self.fp.write('\n')

    def log(self,msg):
        log_msg = f'[{datetime.now()}] {msg}\n'
        self.fp.write(log_msg)
        print(log_msg)

    def binary_search(self):
        return

    def add_project(self,project_name):
        # should check if project exists already-remove if so??
        project_list = self.helper.get_installed_projects()
        if project_name not in project_list:
            # upload project
            self.project = self.helper.post_install_project(project_name)

        # add project to group
        self.helper.put_project_to_group(self.group, self.project)

        # wait? thread block until project added succesfully...?
        # Load deconfliction group
        self.helper.put_load_group(self.group)

    def start(self):
        # this function loops
        # iterate through accel/jerk values
        self.accel = [2.0, 2.0] # [low, high]
        self.jerk = [2.0, 2.0]
        self.done = False

        proj = 'isolated_joint_move_Robot1.zip'

        self.add_project(proj)

        # while self.done == False:
        #
        #
        #     self.done = True






        # clean up stuff
        # delete deconfliction group
        return

def main():
    # If ip address is passed, use it
    if len(sys.argv) != 1:
        ip_addr = str(sys.argv[1])
    else: # Default IP address
        ip_addr = "127.0.0.1"
    print(f'Setting ip address of Realtime Controller to: {ip_addr}')

    fp = open('hub_log.txt','a')
    try:
        task_planner = limitTester(ip_addr,fp)
        task_planner.start()
    finally:
        fp.close()



    # update_limits('')

if __name__ == "__main__":
    main()

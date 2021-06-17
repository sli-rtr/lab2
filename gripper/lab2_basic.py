# this program just moves the arm from hub to hub in the order defined in start()

#!/usr/bin/env python3
import pwd, os, sys
import time, random
from datetime import datetime
from concurrent.futures import ThreadPoolExecutor

from lib.PythonCommander import PythonCommander
from lib.PythonCommanderHelper import PythonCommanderHelper
import lib.CommonOperations as cmn_ops

def LaunchMoveToHub(cmdr,workstate,hub,speed,corner_smoothing,project):
    # Execute this in a thread (MoveToHub function call)
    res,seq = cmdr.MoveToHub(workstate,hub,speed,corner_smoothing,project_name=project)
    code = cmdr.WaitForMove(seq)
    return code

class TaskPlanner():
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
        self.cmdr = PythonCommander(self.ip_addr, 9999)
        self.cmdr.Reconnect()

        code, data = self.cmdr.GetMode()
        if data == 'FAULT':
            code = self.cmdr.ClearFaults()
            if code != self.cmdr.SUCCESS:
                self.log('Failed to clear faults!')
                return

        # Commander helper that communicates with the controller using a REST api
        self.helper = PythonCommanderHelper(self.ip_addr)

        # Request the group info from the group currently loaded on the control panel
        self.group_info = self.helper.get_group_info()
        self.group = None
        for group_name,info in self.group_info.items():
            if info['loaded']:
                self.group = group_name
                self.project_names = info['projects']

        # Nested dictionary that contains all projects information of the form:
        # {project name: {workstates: [str], hubs: [str]}
        self.project_info = self.helper.get_project_info(self.group_info[self.group]['projects'])

        # Call startup sequence which calls InitGroup for each project and then BeginOperationMode
        self.log('Startup sequence...')
        resp = cmn_ops.startup_sequence(self.cmdr,self.project_info,self.group)
        if resp != 0:
            print(f'Startup sequence failed with error code: {resp}')
            return

        # Put each robot on the roadmap
        self.log('Putting robots on the roadmap...')
        move_res = cmn_ops.put_on_roadmap(self.cmdr,self.project_info,self.group,hub='staging')

        if move_res != None:
            if sum(move_res) != 0:
                self.log('Failed to put the robots on the roadmap')
                return

        # Set interupt behavior
        for project_idx in range(0,len(self.project_info)):
            name = self.project_names[project_idx]
            self.cmdr.SetInterruptBehavior(self.replan_attempts,self.timeout,project_name=name)

        self.initialized = True

    # def AcquireTargets(self, cmdr, project_idx):
    #     #This function provides randomized pick positions
    #     #use the following x1 and x2 to work in the range where the robots will work close to each other to test the priority and interlocks
    #     x1 = random.uniform(-0.4, -0.6) # specifies the x-range of the pick positions for the left robot
    #     x2 = random.uniform(-0.4, -0.6) # specifies the x-range of the pick positions for the right robot
    #
    #     y = random.uniform(0.4, 0.6)
    #
    #     pose = []
    #
    #     if project_idx == 0:
    #         pose = [x1, y, 0.05, 0.0,3.14,0.0]
    #     elif project_idx == 1:
    #         pose = [x2, y, 0.05, 0.0,3.14,0.0]
    #
    #     return pose

    # def LaunchPickAndPlace(self, cmdr,workstate, hub, pose, tol, complete_move, complete_move_type,speed,corner_smoothing,project):
    def LaunchPickAndPlace(self, cmdr,workstate, hub, pose, tol, complete_move, complete_move_type,speed,corner_smoothing,project):
        # Execute this in a thread (Pick and Place motion sequence)
        # pick_code = 1
        place_code = 1

        # # move to a position in the road map close to the pick position using MoveToHub
        # res,seq = cmdr.MoveToPose(workstate,pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], tol[0], tol[1], tol[2], tol[3], tol[4], tol[5], complete_move, complete_move_type, speed,corner_smoothing,project_name=project)
        # pick_code = cmdr.WaitForMove(seq)
        # if pick_code != 0:
        #     pick_code = 1
        # else:
        #     pick_code = 0
        #
        # if pick_code == 0:
        # move to the place position using MoveToHub
        # self.log(workstate)
        # self.log(hub)
        # self.log(speed)
        # self.log(corner_smoothing)
        # self.log(project)
        res,seq = cmdr.MoveToHub(workstate,hub,speed,corner_smoothing,project_name=project)
        place_code = cmdr.WaitForMove(seq)
        if place_code != 0:
            place_code = 1
        else:
            place_code = 0

        return place_code   # pick_code + place_code

    def init_logging(self,fp):
        self.fp = fp
        self.fp.write('\n')

    def log(self,msg):
        log_msg = f'[{datetime.now()}] {msg}\n'
        self.fp.write(log_msg)
        print(log_msg)

    def advance_hub(self,project_idx):
        project = self.project_names[project_idx]
        workstate = 'default_state'
        hub_list = self.hubs[project_idx]
        hub_idx = self.hub_idxs[project_idx]

        if hub_idx > len(hub_list)-1:
            self.end_times[project_idx] = time.time()
            self.pick_and_place[project_idx] = False
            self.threads[project_idx] = None
            self.log(f'Project {self.project_names[project_idx]} has finished!')
        else:
            hub = hub_list[hub_idx]
            future = self.executor.submit(LaunchMoveToHub,self.cmdr,workstate,hub,self.speed,self.corner_smoothing,project)
            self.threads[project_idx] = future

    def pick_and_place_part(self,project_idx):
        project = self.project_names[project_idx]
        workstate = 'default_state'
        hub_list = self.hubs[project_idx]
        hub_idx = self.hub_idxs[project_idx]

        if hub_idx > len(hub_list)-1:
            self.end_times[project_idx] = time.time()
            self.pick_and_place[project_idx] = False
            self.threads[project_idx] = None
            self.log(f'Project {self.project_names[project_idx]} has finished!')
        else:
            hub = hub_list[hub_idx]
            future = self.executor.submit(self.LaunchPickAndPlace,self.cmdr,workstate,hub,self.pose,self.tol,self.complete_move,self.complete_move_type, self.speed,self.corner_smoothing,project)
            self.threads[project_idx] = future

    def retract_to_staging(self,project_idx):
        self.log(f'Retracting project {self.project_names[project_idx]} to staging!')
        project = self.project_names[project_idx]
        workstate = 'default_state'
        hub = 'staging'
        future = self.executor.submit(LaunchMoveToHub,self.cmdr,workstate,hub,self.speed,self.corner_smoothing,project)
        self.threads[project_idx] = future

    def start(self):
        '''
        Execute pick and place cycling around the set place positions in hub_1 and hub_2
        '''
        #Check if the initialization was successful and end the program if initialization failed
        if self.initialized == False:
            return

        self.speed = 1.0
        self.corner_smoothing = 0.0
        self.complete_move = 1
        self.complete_move_type = 1

        hub_1 = ['staging', 'low_left', 'low_center', 'low_right', 'front_right', 'front_center', 'front_left', 'front_staging']
        # hub_2 = ['place_1_1', 'place_1_2', 'place_1_3','place_1_4','place_2_1', 'place_2_2', 'place_2_3','place_2_4','staging']
        self.hubs = []

        # Loop through both projects hub lists and match the hub lists to the correct robot
        # This will also check that all hub hubs typed exist, and will prevent 4004 errors
        for project_idx in range(0,len(self.project_info)):
            project_name = self.project_names[project_idx]

            hub_list = [] # Create a list of hub names from the list of dictionaries
            for info in self.project_info[project_name]['hubs']:
                hub_list.append(info['name'])

            skip = False
            for hub in hub_1:
                if hub not in hub_list:
                    self.log(f'hub ({hub}) doesnt exist in project ({project_name})')
                    skip = True
                    break
            if not skip:
                self.hubs.append(hub_1)
            # else:
            #     skip = False
            #     for hub in hub_2:
            #         if hub not in hub_list:
            #             self.log(f'hub ({hub}) doesnt exist in project ({project_name})')
            #             skip = True
            #             break
            #     if not skip:
            #         self.hubs.append(hub_2)

        assert(len(self.hubs)==1),'Failed to assign the hub lists to the loaded projects!'

        self.executor = ThreadPoolExecutor(max_workers=4)
        self.threads = [None]*len(self.project_info)
        self.pick_and_place = [True]*len(self.project_info) # status of the bots. False when back at home
        self.end_times = [None]*len(self.project_info) # cycle time stop watch
        self.hub_idxs = [0]*len(self.project_info) # current index of the hub list
        self.interlocking = [False]*len(self.project_info) # project idx of the robot that had to retreat

        self.log(f'Beginning pick and place task...')
        self.start_time = time.time()

        project_list = list(range(0,len(self.project_info)))
        project_list.reverse()

        # pose = self.AcquireTargets(self.cmdr, project_idx)
        self.pose = [0.5,0.5, 0.05, 0.0,3.14,0.0]
        self.tol = [0.1,0.1,0.1,3.14,3.14,3.14]

        # self.pick_and_place_part(1)
        self.pick_and_place_part(0)

        # While both projects haven't finished their cycles
        while True in self.pick_and_place:
            for project_idx in range(0,len(self.project_info)):
                # pose = self.AcquireTargets(self.cmdr, project_idx)
                if self.pick_and_place[project_idx]: # If the project isn't finished
                    res = self.threads[project_idx].done() # Check if the PickAndPlace call thread terminated
                    if res:
                        code = self.threads[project_idx].result() # Catch the move result and determine what to do

                        if code == self.cmdr.SUCCESS:
                            if not self.interlocking[project_idx]:
                                # If your previous move was not a retraction, that means you completed a pick and place move
                                # and need to increase the hub idx
                                self.log(f'Project {self.project_names[project_idx]} completed move to {self.hubs[project_idx][self.hub_idxs[project_idx]]}!')
                                self.hub_idxs[project_idx] += 1
                            self.pick_and_place_part(project_idx)
                            self.interlocking[project_idx] = False
                        else:
                            # The requested move was blocked by the other robot, so retract to the staging position
                            self.retract_to_staging(project_idx)
                            self.interlocking[project_idx] = True

        for project_idx in range(0,len(self.project_info)):
            hub_time_str = f'{self.project_names[project_idx]} pick and place task took: {self.end_times[project_idx]-self.start_time}'
            self.log(hub_time_str)

def main():
    # If ip address is passed, use it
    if len(sys.argv) != 1:
        ip_addr = str(sys.argv[1])
    else: # Default IP address
        ip_addr = "127.0.0.1"
    print(f'Setting ip address of Realtime Controller to: {ip_addr}')

    fp = open('hub_log.txt','a')
    try:
        task_planner = TaskPlanner(ip_addr,fp)
        task_planner.start()
    finally:
        fp.close()

if __name__=="__main__":
    main()

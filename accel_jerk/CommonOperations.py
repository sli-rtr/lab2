#!/usr/bin/env python3

from rtr_appliance.PythonApplianceCommander import PythonApplianceCommander
import sys
import time

def startup_sequence(cmdr,project_info,group):
    '''
    This function calls InitGroup for each project in a group.
    If all InitGroup calls succeed, the Controller is placed in run mode

    Parameters:
        cmdr (object): Instance of the PythonCommander
        project_info (nested dict): Nested dict returned by PythonCommanderHelper's get_project_info() method
        group (string): Group being controlled

    Returns:
        startup_responses: A dictionary with response codes from all InitGroup calls and the BeginOperationMode call.
                           Of the form {'InitGroupResponses':[int],'BeginOperationResponse':int}
    '''
    code, data = cmdr.GetMode()
    if data == 'OPERATION':
        print('Controller already in operation mode!')
        return 0

    # startup_responses = {'InitGroupResponses':[],'BeginOperationResponse':None}
    init_responses = []
    for project_name,info in project_info.items():
        workstate = info['workstates'][0]
        resp = cmdr.InitGroup(workstate,group_name = group,project_name=project_name)
        init_responses.append(int(resp))
    # startup_responses['InitGroupResponses'] = init_responses

    # If all InitGroup calls returned 0, begin operation mode
    if sum(init_responses) == 0:
        begin_resp = cmdr.BeginOperation()
        # startup_responses['BeginOperationResponse'] = int(begin_resp)
    else:
        print('Could not initialize all projects!')
        for resp in init_responses:
            if resp != 0:
                return resp
    if begin_resp != 0:
        print(f'Begin operation mode call failed with response: {begin_resp}')
        return begin_resp

    return 0

def shutdown(cmdr,group,unload=True):
    '''
    This function takes the controller out of Operation mode and into Config mode.
    Optionally, the group can be unloaded from the control panel

    Parameters:
        cmdr (object): Instance of the PythonCommander
        group (string): Group being controlled
        unload (string): If true, the group will be unloaded from the Control Panel
    '''
    # End operation mode on controller. This will put the controller in config mode
    cmdr.EndOperation()

    # This will unload the group from the controller
    if unload:
        cmdr.TerminateGroup(group)

def put_on_roadmap(cmdr,project_info,group,hub='home'):
    '''
    This function attempts to put all robots back on the roadmap.
    NOTE: In a multi robot scenario one robot may be blocked by the other. If the blocked robot is moved first, the offroad_to_hub call may fail.

    Parameters:
        cmdr (object): Instance of the PythonCommander
        project_info (nested dict): Nested dict returned by PythonCommanderHelper's get_project_info() method
        group (string): Group being controlled
        hub (string): Hub to move the robots to. Default is 'home'

    Returns:
        move_res (list): a list of the move result value from each offroad to hub call. 0 means success
    '''
    code, data = cmdr.GetMode()

    # print(data)
    # if data != 'OPERATION':
    #     print('Put controller in operation mode!')
    #     return

    move_res = []
    for name,info in project_info.items():
        print(name)
        workstate = info['workstates'][0]
        hub_res, hub_seq = cmdr.OffroadToHub(workstate, hub, "low", 240.0, fallback_to_nominal=True, project_name=name, speed=0.1)
        move_res.append(cmdr.WaitForMove(hub_seq,timeout=240.0))

    return move_res


def attempt_fault_recovery(cmdr,project_info,group,hub='home'):
    '''
    This function clears faults on the controller and puts all robots back on the roadmap.
    It is assumed that all projects in the group have a hub named 'home' or that all projects have a hub with the same name.

    Parameters:
        cmdr (object): Instance of the PythonCommander
        project_info (nested dict): Nested dict returned by PythonCommanderHelper's get_project_info() method
        group (string): Group being controlled
        hub (string): Hub name to move the all robots to. Default is 'home'
    '''
    code, data = cmdr.GetMode()
    if data != 'FAULT':
        print('Controller is not in Fault mode!')
        return

    # Clear faults on the RTR Controller
    cmdr.ClearFaults()

    # Restart controller
    startup_sequence(cmdr,project_info,group)

    # Put each robot on the roadmap
    put_on_roadmap(cmdr,project_info,group,hub)

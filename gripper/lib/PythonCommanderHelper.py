#!/usr/bin/env python3

import requests

class ApiError(Exception):
    """An API Error Exception"""
    def __init__(self, status):
        self.status = status
    def __str__(self):
        return f'APIError: status={self.status}'

class PythonCommanderHelper(object):
    get_state = '/api/v1/appliance/state/'
    groups = '/api/v1/groups/'
    installed_proj = '/api/v1/projects/'
    proj_details = '/api/v1/projects/:project/'
    load_group = '/api/v1/groups/load/:group'
    unload_group = '/api/v1/groups/unload/:group'
    config_mode = '/api/v1/appliance/mode/config/'
    clear_faults = '/api/v1/appliance/clear_faults/'
    teleport_robot = '/api/v1/projects/:project/hubs/:hub/'

    def __init__(self,ip_adr):
        self.ip_adr = ip_adr

    def send_get_request(self,extension):
        '''
        This function sends a get request to the passed URL extension and returns the response in JSON form

        Parameters:
            extension (str): the suffix, after http://<ip_address>, of the URL

        Returns:
            resp.json: The REST API's response in JSON form
        '''
        url = f'http://{self.ip_adr}:3000{extension}'
        resp = requests.get(url)
        print(f'\n[INFO] Sent Get request to {url}')

        if resp.status_code != 200:
            # This means something went wrong.
            raise ApiError(f'GET {url} {resp.status_code}')

        return resp.json()

    def send_put_request(self,extension):
        '''
        This function sends a put request to the passed URL extension

        Parameters:
            extension (str): the suffix, after http://<ip_address>, of the URL
        '''
        url = f'http://{self.ip_adr}:3000{extension}'
        resp = requests.put(url)
        print(f'\n[INFO] Sent Put request to {url}')

        if resp.status_code != 200:
            # This means something went wrong.
            raise ApiError(f'PUT {url} {resp.status_code}')

    def get_control_panel_state(self):
        '''
        This function is called with no arguments and returns the current state of the control panel

        Returns:
            state (str): The current state of the control panel
        '''
        state_resp = self.send_get_request(self.get_state)
        state = state_resp['state']

        return state

    def get_group_info(self):
        '''
        This function is called with no arguments and returns a nested dictionary with all the group names. For each group it also returns if
        the group is loaded and the projects installed in that group.

        Returns:
            group_info (nested dict): dictionary of the form {'group name': {'loaded': bool, 'projects': [str]}}
        '''
        groups = self.send_get_request(self.groups)
        group_info = {}
        for group in groups:
            group_name = group['name']
            group_projects = group['projects']
            loaded = group['loaded']
            group_info.update({group_name : {'loaded':loaded,'projects':group_projects}})
        return group_info

    def get_installed_projects(self):
        '''
        This function is called with no arguments and returns a list of all projects installed on the Controller

        Returns:
            project_list (list of strings): A list of strings where each string is the name of a project installed on the Controller
        '''
        projs = self.send_get_request(self.installed_proj)
        project_list = projs['projects']
        return project_list

    def put_load_group(self,group_name):
        '''
        This function is passed a group name and attempts to load that group in the Control Panel

        Parameters:
            group_name (str): name of the group to be loaded in the Control Panel
        '''
        extension,place_holer = self.load_group.split(':')
        extension = extension + group_name + '/'
        self.send_put_request(extension)

    def put_unload_group(self,group_name):
        '''
        This function is passed a group name and attempts to unload that group from the Control Panel

        Parameters:
            group_name (str): name of the group to be unloaded from the Control Panel
        '''
        extension,place_holer = self.unload_group.split(':')
        extension = extension + group_name + '/'
        self.send_put_request(extension)

    def get_project_info(self,projects):
        '''
        This function is passed project names and returns relevant information about each project. It is easiest to call get_group_info() first,
        and then pass the value of the 'projects' key to get_project_info. Then, you never have to worry about spelling, or passing a project name that isn't in a group.

        Parameters:
            projects (list of strings): A list of strings where each string is a project name in the control panel.

        Returns:
            project_info (nested dict): A nested dictionary containing information about every project passed as an argument. The dictionary structure follows:
                                        {'project name': {'workstates': [str], 'hubs': [str]}
        '''
        project_info = {}
        for project in projects:
            # Format string for project info get request
            extension,place_holer = self.proj_details.split(':')
            extension = extension + project + '/'
            resp = self.send_get_request(extension)

            workstates = resp['roadmaps'] # Actually workstates
            hubs = resp['hubs']
            project_info.update({project:{'workstates':workstates,'hubs':hubs}})

        return project_info

    def put_config_mode(self):
        '''
        This function puts the controller in config mode
        '''
        self.send_put_request(self.config_mode)

    def put_teleport_robot(self,project,hub):
        '''
        This function teleports a robot to a specified hub. NOTE: the controller must be in config mode to teleport a simulated robot

        Parameters:
            project (str): The name of the project to teleport
            hub (str): The name of the hub the robot will be teleported to
        '''
        state = self.get_control_panel_state()
        if state != 'CONFIG':
            print('Control Panel must be in CONFIG mode to teleport robot!')
            return

        split_string = self.teleport_robot.split(':')
        prefix = split_string[0]
        project = '%s/hubs/'%(project)
        hub = '%s'%(hub)

        teleport_string = prefix + project + hub
        self.send_put_request(teleport_string)

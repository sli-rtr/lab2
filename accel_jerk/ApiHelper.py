#!/usr/bin/env python3

import requests

class ApiError(Exception):
    """An API Error Exception"""
    def __init__(self, status):
        self.status = status
    def __str__(self):
        return f'APIError: status={self.status}'

class ApiHelper(object):
    get_state = '/api/v1/appliance/state/'
    groups = '/api/v1/groups/'
    installed_proj = '/api/v1/projects/'
    proj_details = '/api/v1/projects/:project/'
    load_group = '/api/v1/groups/load/:group'
    unload_group = '/api/v1/groups/unload/:group'
    config_mode = '/api/v1/appliance/mode/config/'
    clear_faults = '/api/v1/appliance/clear_faults/'
    teleport_robot = '/api/v1/projects/:project/hubs/:hub/'

    group_proj = '/api/v1/groups/:group/projects/:project/'
    group_details = '/api/v1/groups/:group/'

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

    def send_post_request(self,extension,payload):
        '''
        This function sends a post request to the passed URL extension

        Parameters:
            extension (str): the suffix, after http://<ip_address>, of the URL
        '''
        url = f'http://{self.ip_adr}:3000{extension}'

        if payload.get('name'):
            resp = requests.post(url, json=payload)
        else:
            resp = requests.post(url, files=payload)
        print(f'\n[INFO] Sent Post request to {url}')

        if resp.status_code != 200:
            # This means something went wrong.
            raise ApiError(f'POST {url} {resp.status_code}')

    def send_patch_request(self,extension,payload):
        '''
        This function sends a patch request to the passed URL extension

        Parameters:
            extension (str): the suffix, after http://<ip_address>, of the URL
        '''
        url = f'http://{self.ip_adr}:3000{extension}'
        # kwargs['partial'] = True
        resp = requests.patch(url, json=payload)
        print(f'\n[INFO] Sent Patch request to {url}')

        if resp.status_code != 200:
            # This means something went wrong.
            raise ApiError(f'PATCH {url} {resp.status_code}')

    def send_delete_request(self,extension):
        '''
        This function sends a delete request to the passed URL extension

        Parameters:
            extension (str): the suffix, after http://<ip_address>, of the URL
        '''
        url = f'http://{self.ip_adr}:3000{extension}'
        resp = requests.delete(url)
        print(f'\n[INFO] Sent Delete request to {url}')

        if resp.status_code != 200:
            # This means something went wrong.
            raise ApiError(f'DELETE {url} {resp.status_code}')

    def post_create_group(self,group_name):
        '''
        # TODO: description

        Parameters:
        '''
        data = {'name': group_name}
        self.send_post_request(self.groups, data)
        return group_name

    def post_install_project(self, fname):
        '''
        # TODO: description

        Parameters:
        '''
        files = {'project_zip': open(fname, 'rb')}
        self.send_post_request(self.installed_proj, files)

    def put_project_to_group(self,group,project):
        '''
        # TODO: description

        Parameters:
        '''
        split_string = self.group_proj.split(':')
        prefix = split_string[0]
        group = '%s/projects/'%(group)
        project = '%s'%(project)

        group_project_string = prefix + group + project
        self.send_put_request(group_project_string)

    def patch_robot_params(self,project,payload):
        '''
        # TODO: description

        Parameters:
        '''
        # data = {'connection_type': connection}

        # data =
        extension,place_holer = self.proj_details.split(':')
        extension = extension + project + '/'
        # print(extension)
        # print(payload)
        self.send_patch_request(extension, payload) # robot controller
        # self.send_patch_request(extension, {'connection_type':1}) # internal simulated

    def delete_proj_from_group(self,group,project):
        '''
        # TODO: description

        Parameters:
        '''
        split_string = self.group_proj.split(':')
        prefix = split_string[0]
        group = '%s/projects/'%(group)
        project = '%s'%(project)

        group_project_string = prefix + group + project
        self.send_delete_request(group_project_string)

    def delete_project(self,project):
        '''
        # TODO: description

        Parameters:
        '''
        extension,place_holer = self.proj_details.split(':')
        extension = extension + project + '/'

        self.send_delete_request(extension)

    def delete_group(self,group):
        '''
        # TODO: description

        Parameters:
        '''
        extension,place_holer = self.group_details.split(':')
        extension = extension + group + '/'
        self.send_delete_request(extension)

    def get_installed_groups(self):
        '''
        # TODO: description

        Parameters:
        '''
        groups = self.send_get_request(self.groups)
        group_list = [g['GroupName'] for g in groups]
        return group_list






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

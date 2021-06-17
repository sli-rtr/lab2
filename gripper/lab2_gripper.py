# https://dof.robotiq.com/discussion/1962/programming-options-ur16e-2f-85

# from rtr_appliance.PythonApplianceCommander import PythonApplianceCommander as pyASCII

# this program just opens and closes gripper

import robotiq_gripper
import time
import sys

# If ip address is passed, use it
if len(sys.argv) != 1:
    ip_addr = str(sys.argv[1])
else: # Default IP address
    ip_addr = "127.0.0.1"
print(f'Setting ip address of Realtime Controller to: {ip_addr}')


def log_info(gripper):
    print(f"Pos: {str(gripper.get_current_position()): >3}  "
          f"Open: {gripper.is_open(): <2}  "
          f"Closed: {gripper.is_closed(): <2}  ")

print("Creating gripper...")
gripper = robotiq_gripper.RobotiqGripper()
print("Connecting to gripper...")
gripper.connect(ip_addr, 63352)
print("Activating gripper...")
gripper.activate()

print("Testing gripper...")
gripper.move_and_wait_for_pos(255, 255, 255)
log_info(gripper)
gripper.move_and_wait_for_pos(0, 255, 255)
log_info(gripper)

gripper.disconnect()

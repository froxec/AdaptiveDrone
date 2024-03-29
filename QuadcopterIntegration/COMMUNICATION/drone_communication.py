from dronekit import connect
import argparse
from QuadcopterIntegration.Utilities.dronekit_commands import *
import time

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:8401')
args = parser.parse_args()
print('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=57600, wait_ready=False)
vehicle.arm(wait=True)
time.sleep(10)
vehicle.disarm(wait=True)

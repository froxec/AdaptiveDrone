# #code snippet https://gist.github.com/dbaldwin/9185b702091148580fa836c1911f8735
# from dronekit import connect, VehicleMode, LocationGlobalRelative
# from QuadcopterIntegration.Utilities.dronekit_commands import *
# from pymavlink import mavutil
# import time
#
# import argparse
# parser = argparse.ArgumentParser()
# parser.add_argument('--connect', default='localhost')
# args = parser.parse_args()
#
# # Connect to the Vehicle
# print('Connecting to vehicle on: %s' % args.connect)
# vehicle = connect(args.connect, baud=57600, wait_ready=True)
#
# # Function to arm and then takeoff to a user specified altitude
# def arm_and_takeoff(aTargetAltitude):
#
#   print("Basic pre-arm checks")
#   # Don't let the user try to arm until autopilot is ready
#   while not vehicle.is_armable:
#     print(" Waiting for vehicle to initialise...")
#     time.sleep(1)
#
#   while not vehicle.armed:
#     print(" Waiting for arming...")
#     time.sleep(1)
#
#   print("Taking off!")
#   vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
#
#   # Check that vehicle has reached takeoff altitude
#   while True:
#     print(" Altitude: ", vehicle.location.global_relative_frame.alt)
#     #Break and return from function just below target altitude.
#     if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
#       print("Reached target altitude")
#       break
#     time.sleep(1)
#
# # Initialize the takeoff sequence to 20m
# arm_and_takeoff(20)
#
# print("Take off complete")
#
# print("Setting attitude")
# while True:
#   set_attitude(vehicle, 0, 0.5, 0, 0.5)
#   state = get_state(vehicle)
#   print(state)
# # Hover for 10 seconds
# time.sleep(10)
#
# print("Now let's land")
# vehicle.mode = VehicleMode("LAND")
#
# # Close vehicle object
# vehicle.close()
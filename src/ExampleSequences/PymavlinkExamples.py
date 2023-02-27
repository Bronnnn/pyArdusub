from src.Connector import SurfaceComputerToAutopilot as SC2AP
from src.Controller import Commands
import time
from timeit import default_timer
from pymavlink import mavutil
import sys
import math
from pymavlink.quaternion import QuaternionBase

def create_master(timeout_s):
    master = mavutil.mavlink_connection('udpin:127.0.0.1:14551')
    # Make sure the connection is valid
    master.wait_heartbeat(timeout=timeout_s)
    boot_time = time.time()

    return master, boot_time

def disarm(master):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)

    # wait until disarming confirmed
    print("Waiting for the vehicle to disarm")
    master.motors_disarmed_wait()

def change_flight_mode(master, mode):
    # Check if mode is available
    if mode not in master.mode_mapping():
        print('Unknown mode : {}'.format(mode))
        print('Try:', list(master.mode_mapping().keys()))
        sys.exit(1)

    # Get mode ID
    mode_id = master.mode_mapping()[mode]
    # Set new mode
    # master.mav.command_long_send(
    #    master.target_system, master.target_component,
    #    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
    #    0, mode_id, 0, 0, 0, 0, 0) or:
    # master.set_mode(mode_id) or:
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

    while not master.wait_heartbeat().custom_mode == mode_id:
        master.set_mode(mode_id)

        # Wait for ACK command
        # Would be good to add mechanism to avoid endlessly blocking
        # if the autopilot sends a NACK or never receives the message
        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        ack_msg = ack_msg.to_dict()

        # Print the ACK result !
        print(ack_msg)
        print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)

        # Continue waiting if the acknowledged command is not `set_mode`
        if ack_msg['command'] == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            break

def arm(master):
    # arm ardusub
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)
    print("Waiting for the vehicle to arm")
    master.motors_armed_wait()

def standard_request_msg(master, mavlink_msg_id=29, param2=0):
    #param1=29: SCALED_PRESSURE1, param1=137: SCALED_PRESSURE2, param1=143: SCALED_PRESSURE3
    master.mav.command_long_send(
        target_system=master.target_system, #Target System: MAVLink system id of the vehicle (normally "1")
        target_component=master.target_component, #Target Components: Normally "0"
        command=512, #Command: MAV_CMD_REQUEST_MESSAGE
        confirmation=0, #Confirmation
        param1=mavlink_msg_id, #Param 1: check pymavlink/dialects/common for the description and ../EnumEntry for the id or https://mavlink.io/en/messages/common.html#ATTITUDE
        param2=param2, #Param 2: Depends on message requested, see that messages definition for details
        param3=0, param4=0, param5=0, param6=0, param7=0) #Param 3 to 7: not used

def recv_match(master, timeout=1, mavpackettype = 'ATTITUDE'):
    # init timeout
    time_start = default_timer()
    time_passed = 0

    msg = None

    while time_passed<timeout:
        try:
            msg = master.recv_match(type=mavpackettype).to_dict()
            print("received")
            break
        except:
            time_passed = default_timer() - time_start
            print(f"retry: {(timeout-time_passed):.2f}s until timeout.")
        time.sleep(0.1)

    return msg

def set_target_depth(master, boot_time, depth):
    master.mav.set_position_target_global_int_send(
        int(1e3 * (time.time() - boot_time)),  # ms since boot
        master.target_system, master.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        type_mask=(  # ignore everything except z position
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        ), lat_int=0, lon_int=0, alt=depth,  # (x, y WGS84 frame pos - not used), z [m]
        vx=0, vy=0, vz=0,  # velocities in NED frame [m/s] (not used)
        afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
        # accelerations in NED frame [N], yaw, yaw_rate
        #  (all not supported yet, ignored in GCS Mavlink)
    )

def set_target_attitude(roll, pitch, yaw, master, boot_time):
    """ Sets the target attitude while in depth-hold mode.

        'roll', 'pitch', and 'yaw' are angles in degrees.

        """
    master.mav.set_attitude_target_send(
        int(1e3 * (time.time() - boot_time)),  # ms since boot
        master.target_system, master.target_component,
        # allow throttle to be controlled by depth_hold mode
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
        # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
        QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
        0, 0, 0, 0  # roll rate, pitch rate, yaw rate, thrust
    )

def Testsequence():
        print("Connecting to autopilot")
        master, boot_time = create_master(timeout_s=5)
        print(f"\n")

        print("\nSleep 3 seconds")
        time.sleep(3)

        print("\nDisarming")
        disarm(master)
        print('Disarmed!')
        print(f"\n")

        print("\nSleep 3 seconds")
        time.sleep(3)

        # get autopilot version and capabilities
        print("\nRequest 'AUTOPILOT_VERSION'")
        standard_request_msg(master, mavlink_msg_id=148)
        autopilot_version = recv_match(master, mavpackettype="AUTOPILOT_VERSION")
        print(f"Autopilot version: {autopilot_version}")
        # check if autopilot supports commanding position and velocity targets in global scaled integers
        # Source: https://mavlink.io/en/messages/common.html#MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT (maybe use pymavlink isntead of hardcoding)
        MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT = autopilot_version['capabilities']&256
        print(f"MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT: {MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT==256}")

        print("\nSleep 3 seconds")
        time.sleep(3)

        print("\nSet depth hold mode")
        change_flight_mode(master, mode = 'ALT_HOLD')

        print("\nSleep 3 seconds")
        time.sleep(3)

        # arm ardusub
        print("\n!!! Arming !!!")
        arm(master)
        print('!!! Armed !!!')
        print(f"\n")

        print("\nSleep 3 seconds")
        time.sleep(3)

        # get current depth
        print("\nRequest 'GLOBAL_POSITION_INT'")
        Commands.standard_request_msg(master, mavlink_msg_id=33)
        global_position_int = recv_match(master, mavpackettype="GLOBAL_POSITION_INT")

        # update depth
        target_depth_m = -10
        current_depth_mm = global_position_int["alt"]
        current_depth_m = current_depth_mm/1000
        depth_difference_abs_m = abs(target_depth_m - current_depth_m)

        print(f"current depth: {current_depth_m:.2f}m")
        print(f"target depth: {target_depth_m:.2f}m")
        print(f"absolute depth difference: {depth_difference_abs_m:.2f}m")

        print("\nSleep 10 seconds")
        time.sleep(10)

        print(f"\nSet target depth: {target_depth_m}m")
        set_target_depth(master, boot_time, depth=target_depth_m) # can not request the target depth??? does not hold the target depth, even if in depth hold mode

        print("\nSleep 20 seconds")
        time.sleep(20)

        print("\nRequest 'POSITION_TARGET_GLOBAL_INT'")
        standard_request_msg(master, mavlink_msg_id=87)
        position_target_global_int = master.recv_match(master, type="POSITION_TARGET_GLOBAL_INT")
        print(f"Position target global int: {position_target_global_int}")

        print("\nRequest 'GLOBAL_POSITION_INT'")
        standard_request_msg(master, mavlink_msg_id=33)
        global_position_int = SC2AP.recv_match(master, mavpackettype="GLOBAL_POSITION_INT") #could be the correct msg for altitude

        # udpate depth
        target_depth_m = -10
        current_depth_mm = global_position_int["alt"]
        current_depth_m = current_depth_mm/1000
        depth_difference_abs_m = abs(target_depth_m-current_depth_m)

        print(f"\nCurrent depth: {current_depth_mm/1000:.2f}m")
        print(f"Target depth: {target_depth_m:.2f}m")
        print(f"Absolute depth difference: {depth_difference_abs_m:.2f}m")

        print("\nSleep 20 seconds")
        time.sleep(20)

        print("\nGo for a spin")
        # (set target yaw from 0 to 500 degrees in steps of 10, one update per second)
        roll_angle = pitch_angle = 0
        for yaw_angle in range(0, 360, 60):
                set_target_attitude(roll_angle, pitch_angle, yaw_angle, master, boot_time)
                print("\nSleep 10 seconds")
                time.sleep(10)  # wait for a second

        # get current depth
        print("\nRequest 'GLOBAL_POSITION_INT'")
        standard_request_msg(master, mavlink_msg_id=33)
        global_position_int = SC2AP.recv_match(master, mavpackettype="GLOBAL_POSITION_INT")  # could be the correct msg for altitude

        # update depth
        target_depth_m = 0
        current_depth_mm = global_position_int["alt"]
        current_depth_m = current_depth_mm / 1000
        depth_difference_abs_m = abs(target_depth_m - current_depth_m)

        print(f"current depth: {current_depth_m:.2f}m")
        print(f"target depth: {target_depth_m:.2f}m")
        print(f"absolute depth difference: {depth_difference_abs_m:.2f}m")

        print("\nSleep 10 seconds")
        time.sleep(10)

        print(f"\nset target depth: {target_depth_m}m")
        set_target_depth(target_depth_m, master, boot_time)  # can not request the target depth??? does not hold the target depth, even if in depth hold mode

        print("\nSleep 10 seconds")
        time.sleep(10)

        print("\nRequest 'POSITION_TARGET_GLOBAL_INT'")
        standard_request_msg(master, mavlink_msg_id=87)
        position_target_global_int = recv_match(master, type="POSITION_TARGET_GLOBAL_INT")
        print(f"Position target global int: {position_target_global_int}")

        print("Request 'GLOBAL_POSITION_INT'")
        standard_request_msg(master, mavlink_msg_id=33)
        global_position_int = SC2AP.recv_match(master, mavpackettype="GLOBAL_POSITION_INT")  # could be the correct msg for altitude

        # udpate depth using one of the many messages received
        current_depth_mm = global_position_int["alt"]
        current_depth_m = current_depth_mm / 1000
        depth_difference_abs_m = abs(target_depth_m - current_depth_m)

        print(f"current depth: {current_depth_mm / 1000:.2f}m")
        print(f"target depth: {target_depth_m:.2f}m")
        print(f"absolute depth difference: {depth_difference_abs_m:.2f}m")

        # clean up (disarm) at the end
        disarm(master)
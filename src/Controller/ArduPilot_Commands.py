from pymavlink import mavutil
import sys
import time
import math
from pymavlink.quaternion import QuaternionBase

"""
Uses a couple of examples found on https://www.ardusub.com/developers/pymavlink.html
Restructures the examples in functions
"""
def arm(master):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)

    # wait until arming confirmed
    print("Waiting for the vehicle to arm")
    master.motors_armed_wait()
    print('!!! Armed !!!')

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
    print('Disarmed!')

def manual_control(master, x, y, z, r):
    """
    Warning: Because of some legacy workaround, z will work between [0-1000]
    where 0 is full reverse, 500 is no output and 1000 is full throttle.
    x,y and r will be between [-1000 and 1000].
    """
    master.mav.manual_control_send(
        master.target_system,
        x=x,
        y=y,
        z=z,
        r=r,
        buttons=0
    )

def change_flightmode(master, mode='STABILIZE'):
    """
    available modes: ['STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'CIRCLE', 'SURFACE', 'POSHOLD', 'MANUAL']
    """
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

def request_surface_depth_parameter(master):
    # Request parameter
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        b'SURFACE_DEPTH', #parameter id
        -1
    )

def read_all_parameters(master):
    # Request all parameters
    master.mav.param_request_list_send(
        master.target_system, master.target_component
    )
    while True:
        time.sleep(0.01)
        try:
            message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
            print('name: %s\tvalue: %d' % (message['param_id'],
                                           message['param_value']))
        except Exception as error:
            print(error)
            sys.exit(0)

def recv_parameter_value(master):
    # Print received parameter value
    print("parameter value:")
    message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
    print('name: %s\tvalue: %d' %
          (message['param_id'], message['param_value']))

    time.sleep(1)

    return message

def set_surface_depth_parameter(master, depth):
    # Set new parameter value
    master.mav.param_set_send(
        master.target_system, master.target_component,
        b'SURFACE_DEPTH',
        depth,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )

def set_target_depth_ned(depth, master, boot_time):
    """
    Sets the target depth while in depth-hold mode.

    Uses https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT
    adjusted for NED frames https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
    (note, altitude is negative in NED)

    'depth' is technically an altitude, so set as negative meters below the surface
        -> set_target_depth(1.5) # sets target to 1.5m below the water surface.
    """

    master.mav.set_position_target_local_ned_send(int(1e3 * (time.time() - boot_time)),  # ms since boot
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
        ), x=0, y=0, z=-depth,  # (x, y WGS84 frame pos - not used), z [m]
        vx=0, vy=0, vz=0,  # velocities in NED frame [m/s] (not used)
        afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
        # accelerations in NED frame [N], yaw, yaw_rate
        #  (all not supported yet, ignored in GCS Mavlink))
        )

def set_target_depth(depth, master, boot_time):
    """ Sets the target depth while in depth-hold mode.

        Uses https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT

        'depth' is technically an altitude, so set as negative meters below the surface
            -> set_target_depth(-1.5) # sets target to 1.5m below the water surface.

        """

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

def set_target_attitude_rate(roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate, master, boot_time):
    """ Sets the target attitude while in depth-hold mode.

    'roll', 'pitch', and 'yaw' are angles in degrees.

    """
    master.mav.set_attitude_target_send(
        int(1e3 * (time.time() - boot_time)),  # ms since boot
        master.target_system, master.target_component,
        # allow throttle to be controlled by depth_hold mode
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
        # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
        QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]), #roll, pitch, yaw
        roll_rate, pitch_rate, yaw_rate, 0  # roll rate, pitch rate, yaw rate, thrust
    )

def set_target_attitude_rate_2(master, boot_time, yaw_rate):
    """ Sets the target attitude while in depth-hold mode.

    'roll', 'pitch', and 'yaw' are angles in degrees.

    """
    master.mav.set_attitude_target_send(
        body_yaw_rate=yaw_rate,
        body_roll_rate=0,
        body_pitch_rate=0,
        type_mask=mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
        time_boot_ms=boot_time,
        q=QuaternionBase([math.radians(angle) for angle in (0, 0, 0)]), #roll, pitch, yaw,
        target_system=master.target_system,
        target_component=master.target_component,
        thrust=0
    )

def convert_pressure_to_depth(pressure, watertype = 'fresh'):
    """
    converts pressure to depth using the following formulas https://bluerobotics.com/learn/pressure-depth-calculator/
    """
    g = 9.80665
    if watertype == 'fresh':
        p_fresh = 997.0474
        depth = -pressure / (g * p_fresh)

    if watertype == 'salt':
        p_salt = 1023.6
        depth = -pressure / (g * p_salt)

    return depth

def request_frame_global_int(master):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        0,
        1, 0, 0, 0, 0, 0, 0)

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

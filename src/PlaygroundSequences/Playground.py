from src.Connector import SurfaceComputerToAutopilot as SC2AP
from src.Controller import ArduPilot_Commands
from src.Controller import pid_controller
import time
#import ms5837
from timeit import default_timer
import pymavlink
import numpy as np

"""
Links: 
Depth Hold Controller. https://github.com/ArduPilot/ardupilot/blob/fd32425d2495b681a9440f96f0be1c43142fbff5/ArduSub/control_althold.cpp
Alt hold controller should be called at 100 Hz or more (does it mean target depth has to be set so often?)

"""
def hold_depth(master_SC2AP, boot_time, target_depth_m, timeout_s):
        # init timeout
        time_start = default_timer()
        time_passed = 0

        # get current depth
        print("Request 'GLOBAL_POSITION_INT'")
        ArduPilot_Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=33)
        global_position_int = SC2AP.recv_match(master_SC2AP,
                                               mavpackettype="GLOBAL_POSITION_INT")  # could be the correct msg for altitude

        # update depth using one of the many available messages
        current_depth_mm = global_position_int["alt"]
        current_depth_m = current_depth_mm / 1000
        depth_difference_abs_m = abs(target_depth_m - current_depth_m)

        print(f"current depth: {current_depth_m:.2f}m")
        print(f"target depth: {target_depth_m:.2f}m")
        print(f"absolute depth difference: {depth_difference_abs_m:.2f}m")

        # allowed difference between target depth and current depth
        max_depth_difference_m = 0.1

        while time_passed < timeout_s and depth_difference_abs_m > max_depth_difference_m:
                print(f"Set target depth: {target_depth_m}m")
                ArduPilot_Commands.set_target_depth(target_depth_m, master_SC2AP,
                                                    boot_time)  # can not request the target depth??? does not hold the target depth, even if in depth hold mode

                print("Position target global int")
                ArduPilot_Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=87)
                position_target_global_int = master_SC2AP.recv_match(master_SC2AP, type="POSITION_TARGET_GLOBAL_INT")
                print(f"Position target global int: {position_target_global_int}")

                print("Position target local ned")
                ArduPilot_Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=85)
                position_target_local_ned = master_SC2AP.recv_match(master_SC2AP, type="POSITION_TARGET_LOCAL_NED")
                print(f"Position target local ned: {position_target_local_ned}")

                # request scaled pressure
                print("Request 'SCALED_PRESSURE'")
                ArduPilot_Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=29)
                scaled_pressure1 = SC2AP.recv_match(master_SC2AP, mavpackettype="SCALED_PRESSURE")
                # Convert to depth #calculates wrong depth
                # depth = Commands.convert_pressure_to_depth(scaled_pressure1['press_abs']/1000, watertype='salt')
                # print(f"calculated depth: {depth}")

                print("Request 'SCALED_PRESSURE2'")
                ArduPilot_Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=137)
                scaled_pressure2 = SC2AP.recv_match(master_SC2AP, mavpackettype="SCALED_PRESSURE2")

                print("Request 'SCALED_PRESSURE3'")
                ArduPilot_Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=143)
                scaled_pressure3 = SC2AP.recv_match(master_SC2AP, mavpackettype="SCALED_PRESSURE3")

                print("Request 'GLOBAL_POSITION_INT'")
                ArduPilot_Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=33)
                global_position_int = SC2AP.recv_match(master_SC2AP,
                                                       mavpackettype="GLOBAL_POSITION_INT")  # could be the correct msg for altitude

                # Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=62)
                # nav_controller_output = SC2AP.recv_match(master_SC2AP, mavpackettype="NAV_CONTROLLER_OUTPUT")

                # Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=63)
                # global_position_int_cov = SC2AP.recv_match(master_SC2AP, mavpackettype="GLOBAL_POSITION_INT_COV")

                print("Request 'VFR_HUD'")
                ArduPilot_Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=74)
                vfr_hud = SC2AP.recv_match(master_SC2AP,
                                           mavpackettype="VFR_HUD")  # thats even exactly what is displayed in QGC

                # Commands.request_scaled_pressure(master_SC2AP, param1=230)
                # msg = SC2AP.recv_match(master_SC2AP, mavpackettype="ESTIMATOR_STATUS") # not receiving
                # print(msg)

                # Print received parameter value (does not ensure that the next message is actually the pressure...)
                # message = Commands.read_pressure(master=master_SC2AP, num_sensor=1) # thats just the paramter, not the actual measurement!!!
                # print(f"sensor depth: {sensor.depth()}")

                # udpate depth using one of the many messages received
                current_depth_mm = global_position_int["alt"]
                current_depth_m = current_depth_mm / 1000
                depth_difference_abs_m = abs(target_depth_m - current_depth_m)

                print(f"current depth: {current_depth_mm / 1000:.2f}m")
                print(f"target depth: {target_depth_m:.2f}m")
                print(f"absolute depth difference: {depth_difference_abs_m:.2f}m")

                # print the time left for reaching the target depth, before starting to rotate
                time_passed = default_timer() - time_start
                print(f"Get to target depth: {(timeout_s - time_passed):.2f}s until timeout.")
                print("\n")
                time.sleep(0.1)

        print(f"\n")

def Test_manual_control():
        print("Testsequence_SurfaceComputerToAutopilot_w_set_target_position")
        # create connection from surface computer to autopilot
        print("\nConnecting to autopilot")
        master_SC2AP, boot_time = SC2AP.create_master()
        print(f"\n")

        # clean up (disarm)
        print("Inital state")
        ArduPilot_Commands.disarm(master_SC2AP)
        print(f"\n")

        print("Set depth hold mode")
        # ['STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'CIRCLE', 'SURFACE', 'POSHOLD', 'MANUAL']
        ArduPilot_Commands.change_flightmode(master_SC2AP, mode='STABILIZE')

        print("\n!!! Arming. Stay clear !!!")
        time_start = default_timer()
        countdown = 5
        while (default_timer() - time_start < countdown):
                print(round(countdown - (default_timer() - time_start)))
                time.sleep(1)
        # arm ardusub
        ArduPilot_Commands.arm(master_SC2AP)
        print(f"\n")

        # init timer
        time_start = default_timer()
        time_passed = 0
        timeout_s = 5
        print("full speed down")
        while time_passed<timeout_s:
                ArduPilot_Commands.manual_control(master_SC2AP, x=0, y=0, z=0, r=0)
                time_passed = default_timer() - time_start

        # init timer
        time_start = default_timer()
        time_passed = 0
        timeout_s = 5
        print("full speed rotating right")
        while time_passed < timeout_s:
                ArduPilot_Commands.manual_control(master_SC2AP, x=0, y=100, z=500, r=1000)
                time_passed = default_timer() - time_start
        print("full speed forward")
        # init timer
        time_start = default_timer()
        time_passed = 0
        timeout_s = 5
        print("full speed forward")
        while time_passed < timeout_s:
                ArduPilot_Commands.manual_control(master_SC2AP, x=1000, y=0, z=500, r=0)
                time_passed = default_timer() - time_start

        # init timer
        time_start = default_timer()
        time_passed = 0
        timeout_s = 5
        print("full speed sideways right")
        while time_passed < timeout_s:
                ArduPilot_Commands.manual_control(master_SC2AP, x=0, y=1000, z=500, r=0)
                time_passed = default_timer() - time_start

# Cleaned up version of the playground
def Testsequence_SurfaceComputerToAutopilot_w_set_target_position():
        print("Testsequence_SurfaceComputerToAutopilot_w_set_target_position")
        # create connection from surface computer to autopilot
        print("\nConnecting to autopilot")
        master_SC2AP, boot_time = SC2AP.create_master()
        print(f"\n")

        # clean up (disarm)
        print("Inital state")
        ArduPilot_Commands.disarm(master_SC2AP)
        print(f"\n")

        print("Set depth hold mode")
        ArduPilot_Commands.change_flightmode(master_SC2AP, mode='ALT_HOLD')

        print("\n!!! Arming. Stay clear !!!")
        time_start = default_timer()
        countdown = 5
        while (default_timer()-time_start<countdown):
                print(round(countdown - (default_timer()-time_start)))
                time.sleep(1)
        # arm ardusub
        ArduPilot_Commands.arm(master_SC2AP)
        print(f"\n")

        # get autopilot version and capabilities
        print("Request 'AUTOPILOT_VERSION'")
        ArduPilot_Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=148)
        autopilot_version = SC2AP.recv_match(master_SC2AP, mavpackettype="AUTOPILOT_VERSION")
        print(f"Autopilot version: {autopilot_version}")
        # check if autopilot supports commanding position and velocity targets in global scaled integers
        # Source: https://mavlink.io/en/messages/common.html#MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT (maybe use pymavlink isntead of hardcoding)
        MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT = autopilot_version['capabilities']&256
        print(f"MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT: {MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT==256}")
        MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED = autopilot_version['capabilities']&128
        print(f"MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED: {MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED==128}")

        target_depth_m = -10
        timeout_s = 15
        hold_depth(master_SC2AP, boot_time, target_depth_m, timeout_s)

        time_wait_s = 10
        print(f"sleep for {time_wait_s}s to stabilize")
        print(time.sleep(time_wait_s))

        print("set depth hold mode")
        #Commands.change_flightmode(master_SC2AP, mode='GUIDED') #turns by 90 degrees
        #Commands.change_flightmode(master_SC2AP, mode='POSHOLD') # does not turn at all
        ArduPilot_Commands.set_target_depth(target_depth_m, master_SC2AP, boot_time)  # workaround to hold depth
        ArduPilot_Commands.change_flightmode(master_SC2AP, mode='ALT_HOLD')

        # get current attitude
        print("Request 'GLOBAL_POSITION_INT'")
        ArduPilot_Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=33)
        global_position_int = SC2AP.recv_match(master_SC2AP, mavpackettype="GLOBAL_POSITION_INT")  # could be the correct msg for altitude
        heading = global_position_int["hdg"]/100

        # manually calculate the attitude to limit the turning rate
        # (since i didnt manage to get set_target_attitude with rates to work)
        total_turning_degree = 360
        heading_target_degree = heading + total_turning_degree
        stepsize_degree = 0.01
        target_heading_degree = np.arange(heading, heading_target_degree, stepsize_degree)

        print("rotate")
        roll_angle = pitch_angle = 0
        for yaw_angle in target_heading_degree:
                print("Request 'GLOBAL_POSITION_INT'")
                ArduPilot_Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=33)
                global_position_int = SC2AP.recv_match(master_SC2AP, mavpackettype="GLOBAL_POSITION_INT")

                # udpate depth using one of the many messages received
                current_depth_mm = global_position_int["alt"]
                current_depth_m = current_depth_mm / 1000
                depth_difference_abs_m = abs(target_depth_m - current_depth_m)

                print(f"current depth: {current_depth_mm / 1000:.2f}m")
                print(f"target depth: {target_depth_m:.2f}m")
                print(f"absolute depth difference: {depth_difference_abs_m:.2f}m")

                # try to recover the target depth
                if depth_difference_abs_m>0.2:
                        print("recover depth")
                        hold_depth(master_SC2AP, boot_time, target_depth_m, timeout_s)

                #ArduPilot_Commands.set_target_attitude_rate(roll_angle, pitch_angle, yaw_angle, roll_rate=0, pitch_rate=0, yaw_rate=0, master=master_SC2AP, boot_time=boot_time) # does not hold depth
                ArduPilot_Commands.manual_control(master_SC2AP, x=0, y=100, z=500, r=500) # holds depth
                ArduPilot_Commands.set_target_depth(target_depth_m, master_SC2AP, boot_time)
                print(f"current heading: {global_position_int['hdg']/100}")
                print(f"target heading: {heading_target_degree}")
                print(f"absolute heading difference: {abs(global_position_int['hdg']/100-heading_target_degree)}")
                #Commands.set_target_attitude_rate_2(master=master_SC2AP, boot_time=boot_time, yaw_rate=0.1 ) #not working
                time.sleep(0.001)  # wait for a second

        # set depth hold mode
        print("\nset depth hold mode")
        ArduPilot_Commands.change_flightmode(master_SC2AP, mode='ALT_HOLD')

        # init timeout
        time_start = default_timer()
        time_passed = 0
        timeout_s = 20

        # set target depth
        target_depth_m = -0.2

        # get current depth
        print("Request 'GLOBAL_POSITION_INT'")
        ArduPilot_Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=33)
        global_position_int = SC2AP.recv_match(master_SC2AP, mavpackettype="GLOBAL_POSITION_INT")  # could be the correct msg for altitude

        # update depth using one of the many available messages
        current_depth_mm = global_position_int["alt"]
        current_depth_m = current_depth_mm / 1000
        depth_difference_abs_m = abs(target_depth_m - current_depth_m)

        print(f"current depth: {current_depth_m:.2f}m")
        print(f"target depth: {target_depth_m:.2f}m")
        print(f"absolute depth difference: {depth_difference_abs_m:.2f}m")

        # allowed difference between target depth and current depth
        max_depth_difference_m = 0.2

        while time_passed < timeout_s and depth_difference_abs_m > max_depth_difference_m:
                print(f"set target depth: {target_depth_m}m")
                ArduPilot_Commands.set_target_depth(target_depth_m, master_SC2AP, boot_time)  # can not request the target depth??? does not hold the target depth, even if in depth hold mode

                print("Position target global int")
                ArduPilot_Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=87)
                position_target_global_int = master_SC2AP.recv_match(master_SC2AP,
                                                                     type="POSITION_TARGET_GLOBAL_INT")
                print(f"Position target global int: {position_target_global_int}")

                # request scaled pressure
                print("Request 'SCALED_PRESSURE'")
                ArduPilot_Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=29)
                scaled_pressure1 = SC2AP.recv_match(master_SC2AP, mavpackettype="SCALED_PRESSURE")
                # Convert to depth #calculates wrong depth
                # depth = Commands.convert_pressure_to_depth(scaled_pressure1['press_abs']/1000, watertype='salt')
                # print(f"calculated depth: {depth}")

                print("Request 'SCALED_PRESSURE2'")
                ArduPilot_Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=137)
                scaled_pressure2 = SC2AP.recv_match(master_SC2AP, mavpackettype="SCALED_PRESSURE2")

                print("Request 'SCALED_PRESSURE3'")
                ArduPilot_Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=143)
                scaled_pressure3 = SC2AP.recv_match(master_SC2AP, mavpackettype="SCALED_PRESSURE3")

                print("Request 'GLOBAL_POSITION_INT'")
                ArduPilot_Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=33)
                global_position_int = SC2AP.recv_match(master_SC2AP,
                                                       mavpackettype="GLOBAL_POSITION_INT")  # could be the correct msg for altitude

                # Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=62)
                # nav_controller_output = SC2AP.recv_match(master_SC2AP, mavpackettype="NAV_CONTROLLER_OUTPUT")

                # Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=63)
                # global_position_int_cov = SC2AP.recv_match(master_SC2AP, mavpackettype="GLOBAL_POSITION_INT_COV")

                print("Request 'VFR_HUD'")
                ArduPilot_Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=74)
                vfr_hud = SC2AP.recv_match(master_SC2AP,
                                           mavpackettype="VFR_HUD")  # thats even exactly what is displayed in QGC

                # Commands.request_scaled_pressure(master_SC2AP, param1=230)
                # msg = SC2AP.recv_match(master_SC2AP, mavpackettype="ESTIMATOR_STATUS") # not receiving
                # print(msg)

                # Print received parameter value (does not ensure that the next message is actually the pressure...)
                # message = Commands.read_pressure(master=master_SC2AP, num_sensor=1) # thats just the paramter, not the actual measurement!!!
                # print(f"sensor depth: {sensor.depth()}")

                # udpate depth using one of the many messages received
                current_depth_mm = global_position_int["alt"]
                current_depth_m = current_depth_mm / 1000
                depth_difference_abs_m = abs(target_depth_m - current_depth_m)

                print(f"current depth: {current_depth_mm / 1000:.2f}m")
                print(f"target depth: {target_depth_m:.2f}m")
                print(f"absolute depth difference: {depth_difference_abs_m:.2f}m")

                # print the time left for reaching the target depth, before starting to rotate
                time_passed = default_timer() - time_start
                print(f"Get to target depth: {(timeout_s - time_passed):.2f}s until timeout.")
                print("\n")
                time.sleep(0.1)

        # clean up (disarm) at the end
        master_SC2AP.arducopter_disarm()
        master_SC2AP.motors_disarmed_wait()

def init():
        # create connection from surface computer to autopilot
        print("Connecting to autopilot")
        master_SC2AP, boot_time = SC2AP.create_master()
        print(f"\n")

        # clean up (disarm)
        print("Inital state")
        ArduPilot_Commands.disarm(master_SC2AP)
        master_SC2AP.motors_disarmed_wait()
        print(f"\n")

        print("Set depth hold mode")
        ArduPilot_Commands.change_flightmode(master_SC2AP, mode='ALT_HOLD')

        print("\n!!! Arming. Stay clear !!!")
        time_start = default_timer()
        countdown = 5
        while (default_timer() - time_start < countdown):
                print(round(countdown - (default_timer() - time_start)))
                time.sleep(1)
        # arm ardusub
        ArduPilot_Commands.arm(master_SC2AP)
        master_SC2AP.motors_armed_wait()
        print(f"\n")

        return master_SC2AP, boot_time

def check_capabilities(master_SC2AP):
        # get autopilot version and capabilities
        print("Request 'AUTOPILOT_VERSION'")
        ArduPilot_Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=148)
        autopilot_version = SC2AP.recv_match(master_SC2AP, mavpackettype="AUTOPILOT_VERSION")
        print(f"Autopilot version: {autopilot_version}")
        # check if autopilot supports commanding position and velocity targets in global scaled integers
        # Source: https://mavlink.io/en/messages/common.html#MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT (maybe use pymavlink isntead of hardcoding)
        MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT = autopilot_version['capabilities'] & 256
        print(f"MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT: {MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT == 256}")
        MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED = autopilot_version['capabilities'] & 128
        print(f"MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED: {MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED == 128}")

def get_global_position_int(master_SC2AP):
        # get current depth
        print("Request 'GLOBAL_POSITION_INT'")
        ArduPilot_Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=33)
        global_position_int = SC2AP.recv_match(master_SC2AP,
                                               mavpackettype="GLOBAL_POSITION_INT")  # could be the correct msg for altitude

        return global_position_int

# Cleaned up version of the playground
def Testsequence_SurfaceComputerToAutopilot_w_manual_control():
        print("Testsequence_SurfaceComputerToAutopilot_w_manual_control")
        # init communication between surface computer and autopilot
        master_SC2AP, boot_time = init()
        check_capabilities(master_SC2AP)

        # update depth
        global_position_int = get_global_position_int(master_SC2AP)
        target_depth_m = -10
        depth_mm = global_position_int["alt"]
        depth_m = depth_mm / 1000
        depth_difference_abs_m = abs(target_depth_m - depth_m)

        print(f"current depth: {depth_m:.2f}m")
        print(f"target depth: {target_depth_m:.2f}m")
        print(f"absolute depth difference: {depth_difference_abs_m:.2f}m")

        # switch flight mode
        print("Set stabilize mode")
        # ['STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'CIRCLE', 'SURFACE', 'POSHOLD', 'MANUAL']
        ArduPilot_Commands.change_flightmode(master_SC2AP, mode='MANUAL')

        # init depth controller
        depth_controller = pid_controller.depth_controller({"k_p": 3.5, "k_i": 1, "k_d": 0.001})
        max_depth_difference_m = 0.2

        # init timer
        delta_t_s = 0
        #while depth_difference_abs_m > max_depth_difference_m:
        while True:
                depth_controller_output = depth_controller.compute(position=depth_m, target=target_depth_m, delta_t_s=delta_t_s)
                time_start = default_timer()
                print(f"Set manual control z to : {int(depth_controller_output)}")
                ArduPilot_Commands.manual_control(master_SC2AP, x=0, y=0, z=int(depth_controller_output), r=0)

                print("Request 'GLOBAL_POSITION_INT'")
                ArduPilot_Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=33)
                global_position_int = SC2AP.recv_match(master_SC2AP, mavpackettype="GLOBAL_POSITION_INT")

                # udpate depth
                depth_mm = global_position_int["alt"]
                depth_m = depth_mm / 1000
                depth_difference_abs_m = abs(target_depth_m - depth_m)

                print(f"current depth: {depth_mm / 1000:.2f}m")
                print(f"target depth: {target_depth_m:.2f}m")
                print(f"absolute depth difference: {depth_difference_abs_m:.2f}m")
                print(f"\n")

                # time passed, since the function call of depth_controller.compute()
                delta_t_s = default_timer() - time_start

        # switch flight mode
        print("Set stabilize mode")
        # ['STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'CIRCLE', 'SURFACE', 'POSHOLD', 'MANUAL']
        ArduPilot_Commands.change_flightmode(master_SC2AP, mode='POSHOLD')

        # clean up (disarm) at the end
        master_SC2AP.arducopter_disarm()
        master_SC2AP.motors_disarmed_wait()

# Playground
def Testsequence_GroundControlStation():
        #sensor = ms5837.MS5837_30BA()  # Use default I2C bus (1)
        #sensor.init()
        #sensor.read(ms5837.OSR_256)
        #print(sensor.depth())

        # create connection from surface computer to autopilot
        print("Connecting to autopilot")
        master_SC2AP, boot_time = SC2AP.create_master()

        # receive information
        print("Receive packet")
        msg = SC2AP.recv_match(master_SC2AP)
        print(f"Packet received: {msg}")

        # clean up (disarm)
        ArduPilot_Commands.disarm(master_SC2AP)
        master_SC2AP.motors_disarmed_wait()

        # arm ardusub
        ArduPilot_Commands.arm(master_SC2AP)
        master_SC2AP.motors_armed_wait()

        # request current parameter
        print("request current depth")
        ArduPilot_Commands.request_surface_depth_parameter(master_SC2AP)
        # print current parameter
        print("read current depth")
        msg = ArduPilot_Commands.recv_parameter_value(master_SC2AP)
        # set depth
        print("set depth")
        ArduPilot_Commands.set_surface_depth_parameter(master_SC2AP, -1000) # like set target depth but in cm??? Checked it, it has no effect... Why should a external pressure sensor read the depth (when the vehicle is considered at the surface)
        # read ack
        print("read acknowledgment")
        msg = ArduPilot_Commands.recv_parameter_value(master_SC2AP)
        # request parameter value to confirm
        print("request current depth")
        ArduPilot_Commands.request_surface_depth_parameter(master_SC2AP)
        # print new parameter
        print("read current depth")
        msg = ArduPilot_Commands.recv_parameter_value(master_SC2AP)

        # Check if surface depth has any effect
        #while(1):
        #        pass

        # set depth hold mode
        print("set depth hold mode")
        ArduPilot_Commands.change_flightmode(master_SC2AP, mode='ALT_HOLD')

        # Check if surface depth has any effect
        #while(1):
        #        pass

        target_depth_set = False
        while target_depth_set == False:
                # set target depth
                print("set target depth")
                target_depth = -20
                ArduPilot_Commands.set_target_depth(target_depth, master_SC2AP, boot_time) # surface depth shows a different value??? has to be looped in order to have any effect, prefer set surface depth instead???
                # read ack
                print("request current depth")
                ArduPilot_Commands.request_surface_depth_parameter(master_SC2AP)
                # print current parameter
                print("read current depth")
                msg = ArduPilot_Commands.recv_parameter_value(master_SC2AP)
                print(msg)

                # request pressure
                master_SC2AP.mav.param_request_read_send(
                master_SC2AP.target_system, master_SC2AP.target_component,
                b'BARO1_GND_PRESS',
                -1
                )

                # Print received parameter value
                print("parameter value:")
                message = master_SC2AP.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
                print(message)
                print('name: %s\tvalue: %d' %
                      (message['param_id'], message['param_value']))

                # Convert to depth
                pressure = message['param_value']
                g = 9.80665
                p_fresh = 997.0474
                p_salt = 1023.6
                depth = -pressure/(g * p_fresh)
                print(f"calculated depth: {depth}") # pressure not simulated??? same value as the "pressure sensor"??? how to get the real depth, which is necessary to loop until the target depth it is reached, since target_depth does not set surface_depth and has to be looped.
                print(f"sensor depth: {sensor.depth()}")


                #if float(msg['param_value']) != float(target_depth):
                #        print(float(msg['param_value']))
                #        print(float(target_depth))
                #        target_depth_set = False

        if False:
                # (set target yaw from 0 to 500 degrees in steps of 10, one update per second)
                roll_angle = pitch_angle = 0
                for yaw_angle in range(0, 500, 10):
                        ArduPilot_Commands.set_target_attitude(roll_angle, pitch_angle, yaw_angle, master_SC2AP, boot_time)
                        time.sleep(1)  # wait for a second

                # spin the other way with 3x larger steps
                for yaw_angle in range(500, 0, -30):
                        ArduPilot_Commands.set_target_attitude(roll_angle, pitch_angle, yaw_angle, master_SC2AP, boot_time)
                        time.sleep(1)

        # clean up (disarm) at the end
        #master_SC2AP.arducopter_disarm()
        #master_SC2AP.motors_disarmed_wait()

def change_parameter(master):
        # request current parameter
        print("request surface depth")
        ArduPilot_Commands.request_surface_depth_parameter(master)
        # print current parameter
        print("read current surface depth")
        msg = ArduPilot_Commands.recv_parameter_value(master)
        # set depth
        print("set surface depth")
        surface_depth = -30
        ArduPilot_Commands.set_surface_depth_parameter(master,
                                                       surface_depth)  # when at the surface, the bottom of the submarine is at -30cm (thats just a guess. maybe thats what the parameter is good for ...)
        # read ack
        print("read acknowledgment")
        msg = ArduPilot_Commands.recv_parameter_value(master)
        # request parameter value to confirm
        print("request current surface depth")
        ArduPilot_Commands.request_surface_depth_parameter(master)
        # print new parameter
        print("read current surface depth")
        msg = ArduPilot_Commands.recv_parameter_value(master)

        if (msg['param_value'] == surface_depth):
                print("surface depth successfully set")
        else:
                print("error: surface depth not confirmed")
        print(f"\n")

def connect_external_pressure_sensor():
        print("Connecting external pressure sensor")
        ## Pressure sensor (not working or no external pressure sensor exists in the SITL, at least on no simulated I2C connection)
        #sensor = ms5837.MS5837_30BA(bus=1)  # Use default I2C bus (1)
        #sensor.init()
        #sensor.read(ms5837.OSR_256)
        #print(sensor.depth())
        #time.sleep(1)
        #print(f"\n")

def Testsequence(useGroundControlStation = True, useCompanionComputer = False):
    if useGroundControlStation:
        Testsequence_GroundControlStation()

def Testsequence_SurfaceComputerToAutopilot(useSetTargetPosition=True, useManualControl=False):
        if useSetTargetPosition:
                Testsequence_SurfaceComputerToAutopilot_w_set_target_position()
        if useManualControl:
                Testsequence_SurfaceComputerToAutopilot_w_manual_control()
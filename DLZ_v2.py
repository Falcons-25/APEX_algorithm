print("Starting imports...")
from time import sleep, strftime, perf_counter
print(strftime("%Y-%m-%d %H:%M:%S"))
from math import fabs, tan, radians
from threading import Thread#, ThreadError
# print("Importing mav...")
from pymavlink.mavutil import mavlink_connection
# print("Importing cv2...")
from cv2 import waitKey, VideoCapture, aruco#, cvtColor, COLOR_BGR2GRAY
print("Imports complete.")

# adjust in all uses
is_tookoff = False
is_altitude_set = False
is_armed = False
is_guided = False
is_vtol = False
is_at_dlz = False
is_ascending = False
is_descending = False
is_detected = False
is_tracked = False
is_delivered = False
do_capture = False
is_captured = False
# move to numbers and compare in COMMAND_ACK
current_command = None
is_landed = False
completed = False
ended = False
is_current_command_successful = False
zero_alt = 0
alt_calibration = []
target_system = 0
target_component = 0

results = {
    0: "Accepted",
    1: "Temporarily rejected",
    2: "Denied",
    3: "Unsupported",
    4: "Failed",
    5: "In Progress",
    6: "Cancelled",
    7: "long-only",
    8: "int-only",
    9: "Unsupported Frame",
    10: "Permission Denied",
}

modes_plane = {
    0: "Manual",
    1: "Circle",
    2: "Stabilize",
    3: "Training",
    4: "Acro",
    5: "FBWA",
    6: "FBWB",
    7: "Cruise",
    8: "Autotune",
    10: "Auto",
    11: "RTL",
    12: "Loiter",
    13: "Takeoff",
    14: "AVOID_ADSB",
    15: "Guided",
    16: "Initializing",
    17: "QSTABILIZE",
    18: "QHOVER",
    19: "QLOITER",
    20: "QLAND",
    21: "QRTL",
    22: "QAUTOTUNE",
    23: "QACRO",
    24: "THERMAL",
    25: "Loiter to QLAND",
}

modes_copter = {
    0: "Stabilize",
    1: "Acro",
    2: "Alt Hold",
    3: "Auto",
    4: "Guided",
    5: "Loiter",
    6: "RTL",
    7: "Circle",
    # 8: "Position",
    9: "Land",
    # 10: "OF_Loiter",
    11: "Drift",
    13: "Sport",
    14: "Flip",
    15: "AutoTune",
    16: "PosHold",
    17: "Brake",
    18: "Throw",
    19: "Avoid_ADSB",
    20: "Guided_NoGPS",
    21: "Smart_RTL",
    22: "FlowHold",
    23: "Follow",
    24: "ZigZag",
    25: "SystemID",
    26: "Heli_Autorotate",
    27: "Auto RTL",
    28: "Turtle",
    # 29: "",
    # 30: "",
    # 31: "",
}

vtol_states = {
    0: "Undefined",
    1: "Transition to FW",
    2: "Transition to MC",
    3: "MC",
    4: "FW",
}

land_states = {
    0: "Undefined",
    1: "On Ground",
    2: "In Air",
    3: "Takeoff",
    4: "Landing",
}

'''basic support functions'''

def avg(a) -> float:
    global ended
    try:
        if not a: return 0
        return sum(a)/len(a)
    except KeyboardInterrupt:
        print("avg")
        ended = True
        sleep(1)
        print(strftime("%Y-%m-%d %H:%M:%S"))
        return
    except Exception as e:
        print(e)
        print("avg")
        ended = True
        print(strftime("%Y-%m-%d %H:%M:%S"))
        sleep(1)
        return

def mavlink_update():
    global altitude, mode, ended, last_heartbeat, is_armed, is_landed, is_vtol, is_current_command_successful, is_guided, landed_state, vtol_state, zero_alt, is_altitude_set
    # file = open("mavlink_update.txt", "w")
    try:
        altitude = mode = landed_state = vtol_state = 0
        while not ended:
            pkt = connection.recv_match(blocking=True)
            # pkt = connection.recv_match(blocking=True, type=["AHRS2", "HEARTBEAT", "EXTENDED_SYS_STATE", "COMMAND_ACK"])
            if pkt:
                # file.write(str(pkt)+'\n')
                if pkt.get_type()=="GLOBAL_POSITION_INT": # Use GLOBAL_POSITION_INT for GPS and if not available
                    altitude = float(pkt.alt)/1000
                    if (n:=len(alt_calibration))<10:
                        print("Reading altitude...", n)
                        alt_calibration.append(altitude)
                        if len(alt_calibration)==10:
                            print("Calibrating altitude...")
                            zero_alt = round(avg(alt_calibration), 3)
                            print("Altitude:", zero_alt)
                            is_altitude_set = True
                        else: is_altitude_set = False
                    else:
                        altitude = round(altitude - zero_alt, 3)
                        # if altitude <= 0.1:
                            # altitude = 50 * (altitude + 0.1)
                    # get lat, long; compare with dlz to verify reached
                elif pkt.get_type()=="HEARTBEAT":
                    connection.mav.heartbeat_send(6, 8, 0, 0, 0)
                    if pkt.type==6: continue
                    last_heartbeat = pkt
                    basemode = int(pkt.base_mode)
                    mode = int(pkt.custom_mode)
                    # if basemode&(1<<7)==(1<<7):       # REMOVE BEFORE FLIGHT
                        # is_armed = True
                    is_armed = True
                    if mode == 4: is_guided = True
                    elif mode == 9: is_guided = True
                    else:
                        is_guided = False
                        print(mode, modes_copter[mode])
                elif pkt.get_type() == "SET_POSITION_TARGET_LOCAL_NED":
                    print(pkt)
                elif pkt.get_type() == "EXTENDED_SYS_STATE":
                    landed_state = land_states[int(pkt.landed_state)]
                    vtol_state = vtol_states[int(pkt.vtol_state)]
                    if landed_state == "On Ground":
                        # check if took off
                        is_landed = True
                    else: is_landed = False
                    is_landed = False             # ONLY BEFORE FLIGHT
                    if pkt.vtol_state == 3:
                        is_vtol = True
                    else:
                        is_vtol = False
                elif pkt.get_type() == "COMMAND_ACK":
                    print(f"{current_command}: {results[pkt.result]}")
                    if pkt.result == 0:
                        is_current_command_successful = True
                    else:
                        is_current_command_successful = False
            print("AMTLV", altitude, mode, landed_state, vtol_state)
    except KeyboardInterrupt:
        print("mavlink_update")
        ended = True
        print(strftime("%Y-%m-%d %H:%M:%S"))
        sleep(1)
        return
    except Exception as e:
        print(e)
        print("mavlink_update")
        ended = True
        print(strftime("%Y-%m-%d %H:%M:%S"))
        sleep(1)
        return

'''operations'''

def reach_dlz():
    '''
    0b00 - not in range
    0b01 - in range
    0b10 - transitioning
    0b11 - reached and transitioned
    '''
    # check gps if arrived, transitioned, gspeed -> 0
    # if (dlz_lat - lat < range) and (dlz_long - long < range):
        # is_at_dlz = True
    pass

def detect_dlz():
    global is_detected, is_tracked, ended, is_tookoff
    # if is_guided: is_tookoff = True
    # if is_landed: is_tookoff = False
    count = 0
    tracker = 0
    try:
        while not ended:
            # frame = cap.capture_array("main")
            ret, frame = cap.read()
            if (not ret) or (ret is None):
                is_detected = False
                count += 1
                if count>24:
                    print("Cannot read from camera.")
                    ended = True
                    sleep(1)
                    return
                continue
            corners, ids, _ = aruco.detectMarkers(frame, aruco_dict)
            if ids is not None:
                try:
                    i = list(ids).index(current_payload)
                except ValueError:
                    is_detected = False
                    continue
                if ids[i] == current_payload:
                    print(f"Detected ArUco marker {ids[i][0]}")
                    # print(ids)
                    if ids[i]==current_payload:
                        # track
                        # print(corners[i][0])
                        is_detected = True
                        cX = avg([c[0][0] for c in corners[i]])
                        cY = avg([c[0][1] for c in corners[i]])
                        print("Detected centre:", cX, cY)
                        if not is_guided:
                            print("Detection NOT GUIDED", last_heartbeat)
                            tracker = 0
                            is_detected = True
                            is_tracked = False
                            continue
                        if (fabs((cX-midX)/midX)<0.3*midX) and (fabs((cY-midY)/midY)<0.3*midY):
                            is_detected = True
                            tracker += 1
                            print("Detection Tracker:", tracker)
                        if tracker>4:
                            is_detected = True
                            is_tracked = True
                            return
                        else:
                            continue
                else:
                    print("Payload marker not detected.")
                    is_tracked = False
                    is_detected = False
                    tracker = 0
            if waitKey(1) & 0xFF == ord('q'):
                print("Exiting...")
                return
    except KeyboardInterrupt:
        print("detect_dlz")
        ended = True
        print(strftime("%Y-%m-%d %H:%M:%S"))
        sleep(1)
        return
    except Exception as e:
        print(e)
        print("detect_dlz")
        ended = True
        print(strftime("%Y-%m-%d %H:%M:%S"))
        sleep(1)
        return

def land_dlz():
    global is_detected, is_tracked, is_ascending, is_descending, is_at_dlz, is_landed, current_command, is_current_command_successful, ended
    # check = True
    tracker = 0
    counter = 0
    xbar = []
    ybar = []
    try:
        while not ended:
            ret, frame = cap.read()
            if not ret:
                is_detected = False
                is_ascending = is_descending = False
                counter += 1
                if counter>24:
                    print(f"({counter}) Cannot read from camera.")
                    ended = True
                    return
                continue
            start_time = perf_counter()
            corners, ids, _ = aruco.detectMarkers(frame, aruco_dict)
            if ids is not None:
                try:
                    i = list(ids).index(current_payload)
                except ValueError:
                    is_detected = False
                    print("FPS:", 1/(perf_counter()-start_time))
                    continue
                if ids[i]==current_payload:
                    print(f"Detected ArUco marker {ids[i]}")
                    # connection.mav.command_long_send(
                        # target_system, target_component, 183, 0, 5, 1000, 0, 0, 0, 0, 0)
                    is_detected = True
                    # track
                    cX = avg([c[0][0] for c in corners[i]])
                    cY = avg([c[0][1] for c in corners[i]])
                    # landing_target
                    distX = round(altitude * tan(radians(((cX-midX)/midX)*fovX/2)), 2)
                    distY = round(altitude * tan(radians(((cY-midY)/midY)*fovY/2)), 2)
                    print(distX, -distY)
                    if len(xbar)<0:
                        if is_guided:
                            xbar.append(distX)
                            print("Landing XBar:", len(xbar))
                        else:
                            print("Landing NOT GUIDED.", last_heartbeat)
                            xbar[:] = []
                            ybar[:] = []
                    else:
                        xbar[:] = []
                        ybar[:] = []
                        if is_guided:
                            margin = altitude*tan(radians(30))
                            print(fabs(distX), margin, fabs(distY), margin)
                            if (fabs(distX)<=margin) and (fabs(distY)<=margin):
                                tracker += 1
                                print("Landing Tracker:", tracker)
                            else:
                                tracker = 0
                                is_tracked = False
                                print("Detected, not tracked, ascending.")
                                msg = connection.mav.set_position_target_local_ned_encode(
                                    target_system, target_component, 0, 12, 0b100111111000,
                                    -distY/2, distX/2, -altitude/5, 0, 0, 0, 0, 0, 0, 0, 0)
                                # connection.mav.command_long_send(
                                    # target_system, target_component, 183, 0, 5, 1000, 0, 0, 0, 0, 0)
                                print(msg)
                                connection.mav.send(msg)
                                is_ascending = True
                                is_descending = False
                            if tracker>5:
                                print("Landing Tracker:", tracker)
                                is_tracked = True
                                # descend
                                if altitude>0.3:    # min focus dist + ~0.1
                                    # or send velocity
                                    print(altitude)
                                    msg = connection.mav.set_position_target_local_ned_encode(
                                        target_system, target_component, 0, 12, 0b100111111000,
                                        -distY, distX, altitude/2, 0, 0, 0, 0, 0, 0, 0, 0)
                                    # msg = connection.mav.set_position_target_local_ned_encode(
                                        # target_system, target_component, 0, 12, 0b000000000000,
                                        # 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
                                    # works, verified
                                    # connection.mav.command_long_send(
                                        # target_system, target_component, 183, 0, 5, 1000+1000*(tracker%2), 0, 0, 0, 0, 0)
                                    print(msg)
                                    connection.mav.send(msg)
                                    is_descending = True
                                    is_ascending = False
                                else:
                                    # land
                                    # current_command = "MAV_CMD_NAV_LAND"
                                    current_command = "SET_POSITION_LOCAL_NED"
                                    is_current_command_successful = False
                                    print("Sending land command.")
                                    print(altitude)
                                    # connection.mav.command_long_send(
                                        # target_system, target_component, 21, 0, 0, 0, 0, 0, 0, 0, 0)
                                    msg = connection.mav.set_position_target_local_ned_encode(
                                        target_system, target_component, 0, 12, 0b100111000111, 0, 0, 0, -distY, distX, altitude, 0, 0, 0, 0, 0)
                                    print(msg)
                                    connection.mav.send(msg)
                                    current_command = "MAV_CMD_NAV_LAND"
                                    sleep(2)
                                    # check in mav_update
                                    connection.mav.command_long_send(
                                        target_system, target_component, 21, 0, 0, 0, 0, 0, 0, 0, 0)
                                    ack = connection.recv_match(blocking=True, type="COMMAND_ACK", timeout=1)
                                    if ack and ack.result==0:
                                        is_descending = True
                                        is_current_command_successful = True
                                        is_landed = True
                                    elif mode==9:
                                        print("Landing")
                                        is_descending = True
                                        is_landed = True
                                    else:
                                        if ack is None:
                                            pass
                                        else:
                                            print("Could not land:", results[ack.result])
                                            is_current_command_successful = False
                                    ended = True
                                    print("FPS:", 1/(perf_counter()-start_time))
                                    sleep(0.5)
                                    return
                                xbar[:] = []
                                ybar[:] = []
                                print("Landing xbar:", len(xbar))
                                print("FPS:", 1/(perf_counter()-start_time))
                        else:
                            print("Landing NOT GUIDED.", last_heartbeat)
                            print("FPS:", 1/(perf_counter()-start_time))
                else:
                    is_detected = False
                    # reach_dlz
                    is_at_dlz = False
                    print("FPS:", 1/(perf_counter()-start_time))
            else:
                is_detected = False
                # reach_dlz
                is_at_dlz = False
                print("FPS:", 1/(perf_counter()-start_time))
            if waitKey(1) & 0xFF == ord('q'):
                print("Exiting...")
                return
    except KeyboardInterrupt:
        print("land_dlz")
        ended = True
        print(strftime("%Y-%m-%d %H:%M:%S"))
        sleep(1)
        cap.release()
        return
    except Exception as e:
        print(e)
        print("land_dlz")
        ended = True
        print(strftime("%Y-%m-%d %H:%M:%S"))
        sleep(1)
        cap.release()
        return

def release_payload():
    # deliver but doesn't need landing, if unable to detect code
    pass
    return

def deliver_payload():
    global is_delivered, ended, is_landed, current_command, is_current_command_successful, is_detected, is_ascending, is_descending, is_tracked, do_capture, is_captured, is_armed
    try:
        if not is_guided: return
        if is_delivered: return
        if is_landed:
            print("Landed, actuating servo.")
            current_command = "MAV_CMD_DO_SET_SERVO"
            # current_command = "MAV_CMD_DO_GRIPPER"
            is_current_command_successful = False
            print("Setting servo to deliver.")
            connection.mav.command_long_send(
                target_system, target_component,
                # 211, 0, 1, 0, 0, 0, 0, 0, 0)       # doesn't work on Copter
                183, 0, 5, 1000, 0, 0, 0, 0, 0)
            ack = connection.recv_match(blocking=True, type="COMMAND_ACK", timeout=1)
            if ack and ack.result==0:
                print("Servo set to 1000")
                is_delivered = True
            elif is_current_command_successful: is_delivered = True
            elif not is_current_command_successful:
                print(f"{current_command}: No ACK received.")
                is_delivered = False
        else: print("Deliver Not landed.")
        is_landed = False
        is_detected = False
        is_ascending = False
        is_descending = False
        is_tracked = False
        do_capture = True
        is_captured = False
        # REMOVE BEFORE FLIGHT (maybe)
        is_armed = True
    except KeyboardInterrupt:
        print("deliver_payload")
        ended = True
        print(strftime("%Y-%m-%d %H:%M:%S"))
        sleep(1)
        return
    except Exception as e:
        print(e)
        print("deliver_payload")
        ended = True
        print(strftime("%Y-%m-%d %H:%M:%S"))
        sleep(1)
        return

def takeoff():
    global is_armed, is_guided, current_command, is_current_command_successful, ended, is_ascending
    # takeoff again and detect payload
    try:
        while not is_guided:
            current_command = "MAV_CMD_DO_SET_MODE"
            is_current_command_successful = False
            connection.mav.command_long_send(
                target_system, target_component, 176, 0, 9, 4, 0, 0, 0, 0, 0)
            ack = connection.recv_match(blocking=True, type="COMMAND_ACK", timeout=1)
            if ack and ack.result==0:
                print("Guided.")
                is_guided = True
                is_current_command_successful = True
            else:
                print("Could not set guided mode.")
                is_guided = False
                is_current_command_successful = False
        else:
            print("Is GUIDED.")
        if not is_armed:
            current_command = "MAV_CMD_COMPONENT_ARM_DISARM"
            is_current_command_successful = False
            msg = connection.mav.command_long_send(
                target_system, target_component, 400, 0, 1, 0, 0, 0, 0, 0, 0)
            connection.mav.send(msg)
            ack = connection.recv_match(blocking=True, type="COMMAND_ACK", timeout=1)
            if ack and ack.result==0:
                print("Armed.")
                is_armed = True
                is_current_command_successful = True
            else:
                print("Could not arm.")
                print(last_heartbeat)
                is_current_command_successful = False
        else:
            print("Is ARMED.")
        if is_armed and is_guided:
            current_command = "MAV_CMD_NAV_TAKEOFF"
            is_current_command_successful = False
            msg = connection.mav.command_long_send(
                target_system, target_component, 22, 0, 0, 0, 0, 0, 0, 0, 0)
                # target_system, target_component, 22, 0, 0, 0, 0, float("nan"), dlz_lat, dlz_long, detection_alt)
            connection.mav.send(msg)
            ack = connection.recv_match(blocking=True, type="COMMAND_ACK", timeout=1)
            if ack and ack.result==0:
                print("Takeoff initiated.")
                is_current_command_successful = True
                is_ascending = True
            else:
                print("Could not takeoff.")
                is_current_command_successful = False
                is_ascending = False
    except KeyboardInterrupt:
        print("takeoff")
        ended = True
        print(strftime("%Y-%m-%d %H:%M:%S"))
        sleep(1)
        return
    except Exception as e:
        print(e)
        print("takeoff")
        ended = True
        print(strftime("%Y-%m-%d %H:%M:%S"))
        sleep(1)
        return

# def detect_payload():
#     tracker = 0
#     try:
#         while not ended:
#             ret, frame = cap.read()
#             if ret:
#                 corners, ids, _ = aruco.detectMarkers(frame, aruco_dict)
#                 if ids:
#                     for i in range(len(ids)):
#                         # print(f"Detected ArUco marker {ids[i]}")
#                         if ids[i]==last_payload:
#                             tracker+=1
#                         if tracker>4:
#                             return 0b101
#                         # land and actuate to 2000 for pickup
#                         # 1000 before landing to keep open (indep of RC input)
#                 else:
#                     return 0b100
#             if waitKey(1) & 0xFF == ord('q'):
#                 print("Exiting...")
#                 return 0b100
#     except KeyboardInterrupt:
#         ended = True
#         print(strftime("%Y-%m-%d %H:%M:%S"))
#         sleep(1)
#         return
#     except Exception as e:
#         print(e)
#         print("detect_payload")
#         ended = True
#         print(strftime("%Y-%m-%d %H:%M:%S"))
#         sleep(1)
#         return
#     return

def pickup_payload():
    # pick up payload and return True when confirmed
    global is_delivered, current_command, is_current_command_successful, ended
    try:
        current_command = "MAV_CMD_DO_SET_SERVO"
        is_current_command_successful = False
        msg = connection.mav.command_long_send(
            target_system, target_component,
            # 211, 0, 1, 1, 0, 0, 0, 0, 0)
            183, 0, 5, 2000, 0, 0, 0, 0, 0)
        ack = connection.recv_match(blocking=True, type="COMMAND_ACK", timeout=3)
        if ack and ack.result==0:
            is_delivered = True
            is_current_command_successful = True
        is_delivered = False
    except KeyboardInterrupt:
        print("pickup_payload")
        ended = True
        print(strftime("%Y-%m-%d %H:%M:%S"))
        sleep(1)
        return
    except Exception as e:
        print(e)
        print("pickup_payload")
        ended = True
        print(strftime("%Y-%m-%d %H:%M:%S"))
        sleep(1)
        return
    return

def capture_payload():
    global is_delivered, ended, is_landed, current_command, is_current_command_successful, is_detected, is_ascending, is_descending, is_tracked, do_capture, is_captured, is_armed
    try:
        if not is_guided: return
        if is_delivered: return
        if is_landed:
            print("Landed, actuating servo.")
            current_command = "MAV_CMD_DO_SET_SERVO"
            is_current_command_successful = False
            print("Setting servo to capture.")
            connection.mav.command_long_send(
                target_system, target_component,
                # 211, 0, 1, 0, 0, 0, 0, 0, 0)       # doesn't work on Copter
                183, 0, 5, 2000, 0, 0, 0, 0, 0)
            ack = connection.recv_match(blocking=True, type="COMMAND_ACK", timeout=3)
            if ack and ack.result==0:
                print("Servo set to 1000")
                is_captured = True
            elif is_current_command_successful: is_captured = True
            elif not is_current_command_successful:
                print(f"{current_command}: No ACK received.")
        is_landed = False
        is_detected = False
        is_ascending = False
        is_descending = False
        is_tracked = False
        # do_capture = True
        is_delivered = True
        # REMOVE BEFORE FLIGHT (maybe)
        # is_armed = True
    except KeyboardInterrupt:
        print("capture_payload")
        ended = True
        print(strftime("%Y-%m-%d %H:%M:%S"))
        sleep(1)
        return
    except Exception as e:
        print(e)
        print("deliver_payload")
        ended = True
        print(strftime("%Y-%m-%d %H:%M:%S"))
        sleep(1)
        return
    return

def return_to_base():
    global current_command, is_current_command_successful, ended
    '''
    0b000 - disarmed
    0b001 - armed
    0b010 - t/o started
    0b011 - t/o complete
    0b100 - in transition
    0b101 - in fw
    0b111 - rtb
    0b1000 - landing
    '''
    # t/o and transition to vtol to return to base
    try:
        if state==0b000:
            current_command = "MAV_CMD_COMPONENT_ARM_DISARM"
            print("Arming.")
            msg = connection.mav.command_long_send(
                target_system, target_component,
                400, 0, 1, 0, 0, 0, 0, 0, 0)
            ack = connection.recv_match(blocking=True, type="COMMAND_ACK", timeout=1)
            if ack and ack.result==0:
                print("Armed.")
                state = 0b001
                is_current_command_successful = True
            else:
                print("Disarmed.")
                is_current_command_successful = False
                return 0b000
        elif state==0b001:
            current_command = "MAV_CMD_NAV_TAKEOFF"
            msg = connection.mav.command_long_send(
                target_system, target_component,
                22, 0, 0, 0, 0, 0, 0, 0, 0)
            ack = connection.recv_match(blocking=True, type="COMMAND_ACK", timeout=1)
            if ack and ack.result==0:
                print("Takeoff initiated.")
                state = 0b010
                is_current_command_successful = True
            else:
                print("Could not takeoff.")
                is_current_command_successful = False
                return 0b000
        elif state==0b010:
            # transition to fw
            # not currently working because switches to manual
            current_command = "MAV_CMD_DO_VTOL_TRANSITION"
            msg = connection.mav.command_long_send(
                target_system, target_component,
                3000, 0, 1, 0, 0, 0, 0, 0, 0)
            ack = connection.recv_match(blocking=True, type="COMMAND_ACK", timeout=1)
            if ack and ack.result==0:
                print("Transitioning to FW.")
                state = 0b100
                is_current_command_successful = True
            elif ack and ack.result==5:
                print("FW Transition in progress.")
                state = 0b101
                is_current_command_successful = True
            else:
                print("Could not transition.")
                is_current_command_successful = False
                return 0b010
        elif state==0b100:
            # check if transition complete
            # if complete: return 0b101
            # else: return 0b100
            pass
        elif state==0b101:
            current_command = "MAV_CMD_NAV_RETURN_TO_LAUNCH"
            msg = connection.mav.command_long_send(
                target_system, target_component,
                20, 0, 0, 0, 0, 0, 0, 0, 0)
            ack = connection.recv_match(blocking=True, type="COMMAND_ACK", timeout=1)
            if ack and ack.result==0:
                print("Returning to base.")
                state = 0b111
                is_current_command_successful = True
            else:
                print("RTB command failed.")
                is_current_command_successful = False
                return 0b101
        # check gps if arrived
        # if arrived:
        # current_command = "MAV_CMD_NAV_LAND"
        # msg = connection.mav.command_long_send(
        #     target_system, target_component,
        #     mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
        # ack = connection.recv_match(blocking=True, type="COMMAND_ACK", timeout=1)
        # if ack and ack.result==0:
        #     print("Landing initiated.")
        #     state = 0b1000
            # is_current_command_successful = True
        # else:
        #     print("Could not land.")
            # is_current_command_successful = False
        #     return 0b111
    except KeyboardInterrupt:
        print("return_to_base")
        ended = True
        print(strftime("%Y-%m-%d %H:%M:%S"))
        sleep(1)
        return
    except Exception as e:
        print(e)
        print("return_to_base")
        ended = True
        print(strftime("%Y-%m-%d %H:%M:%S"))
        sleep(1)
        return
    return

def setup():
    global cap, connection, altitude, midX, midY, fovX, fovY, is_landed, mode, current_payload, last_payload, aruco_dict, ended, completed, current_command, is_current_command_successful, is_tookoff
    altitude =  0
    mode = -1
    print("Setting up camera...")
    cap = VideoCapture(0)
    midX = 0
    midY = 0
    ret = False
    while not ret:
        ret, frame = cap.read()
        if ret:
            midX = frame.shape[1]//2
            midY = frame.shape[0]//2
    fovX = 67.38    # webcam
    fovY = 53.13    # webcam
    print("Camera set up.")
    print("Setting up detector...")
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
    # parameters = aruco.DetectorParameters()
    print("Detector set up.")
    print("Setting up MAVLink...")
    connection = mavlink_connection("/dev/serial0", baud=57600)
    # connection = mavlink_connection("COM14", baud=57600)
    ack = connection.recv_match(blocking=True, type="HEARTBEAT", timeout=5)
    print(f"Heartbeat received:\n{ack}")
    if ack is None:
        print("No ack")
        ended = True
    current_command = "MAV_CMD_SET_MESSAGE_INTERVAL"
    is_current_command_successful = False
    connection.mav.command_long_send(
        target_system, target_component, 511, 0, 33, 500000, 0, 0, 0, 0, 0, 0)
    ack = connection.recv_match(blocking=True, type="COMMAND_ACK")
    if ack and (ack.result==0):
        print("GLOBAL_POSITION_INT set to 2Hz")
        is_current_command_successful = True
    print(ack)
    is_current_command_successful = False
    connection.mav.command_long_send(
        target_system, target_component, 511, 0, 74, 500000, 0, 0, 0, 0, 0, 0)
    ack = connection.recv_match(blocking=True, type="COMMAND_ACK")
    if ack and (ack.result==0):
        print("VFR_HUD set to 2Hz")
        is_current_command_successful = True
    print(ack)
    is_current_command_successful = False
    connection.mav.command_long_send(
        target_system, target_component, 511, 0, 178, 500000, 0, 0, 0, 0, 0, 0)
    ack = connection.recv_match(blocking=True, type="COMMAND_ACK")
    if ack and (ack.result==0):
        print("AHRS2 set to 2Hz")
        is_current_command_successful = True
    print(ack)
    is_current_command_successful = False
    connection.mav.command_long_send(
        target_system, target_component, 511, 0, 245, 1000000, 0, 0, 0, 0, 0, 0)
    ack = connection.recv_match(blocking=True, type="COMMAND_ACK")
    if ack and (ack.result==0):
        print("EXTENDED_SYS_STATE set to 1Hz")
        is_current_command_successful = True
    print(ack)
    mav_thread = Thread(target=mavlink_update)
    mav_thread.start()
    print("MAVLink thread started.")
    print("MAVLink set up.")
    last_payload = [1]
    current_payload = [1]
    # is_tookoff = False
    is_tookoff = True
    print("Definitions complete.")
    current_command = "MAV_CMD_DO_SET_SERVO"
    is_current_command_successful = False
    connection.mav.command_long_send(
        target_system, target_component, 183, 0, 5, 2000, 0, 0, 0, 0, 0)
    sleep(1)

if __name__ == "__main__":
    setup()
    print("Setup complete")
    cont = False
    try:
        while not completed:
            # do mission
            # while not is_at_dlz:
                # reach_dlz()
            while not is_altitude_set:
                pass
            while not is_tracked:       # if is_detected and not is_tracked: reach here?
                print("Ended:", ended)
                if ended:
                    print("Ending")
                    break
                print("Detecting DLZ now:")
                detect_dlz()
            if ended: break
            if cont or completed: continue
            print("Tracked. PROCEEDING")
            print("DLZ Detected, landing.")
            while not is_landed:
                if ended: break
                if not is_guided:
                    print("Not guided", last_heartbeat)
                    cont = True
                    break
                cont = False
                land_dlz()
                if is_detected and not is_tracked: cont = True
                else: cont = False
            if ended: break
            if cont or completed: continue
            print("Landed. PROCEEDING")
            if not ended:
                if is_landed:
                    while not is_delivered:
                        print("Delivering payload...")
                        deliver_payload()
                        if is_delivered:
                            cont = False
                            completed = True
                    if cont or completed: continue
            continue
            # print("Starting takeoff")
            # takeoff()
            # print("Sending takeoff again")
            # while (not is_ascending) or is_landed:
                # takeoff()
            # while not is_tracked:
                # if ended: break
                # print("Detecting DLZ now:")
                # detect_dlz()
            # if cont or completed: continue
            # print("DLZ Detected, landing.")
            # while not is_landed:
                # if ended: break
                # if not is_guided:
                    # print("Not guided", last_heartbeat)
                    # cont = True
                    # break
                # cont = False
                # land_dlz()
                # if is_detected and not is_tracked: cont = True
                # else: cont = False
            # if cont or completed: continue
            # if not ended:
                # if is_landed:
                    # while not is_captured:
                        # print("Capturing payload...")
                        # pickup_payload()
                        # if is_captured:
                            # cont = False
                            # completed = True
                    # if cont or completed: continue
            # else: completed = cont = True
            # while not is_captured:
                # pickup_payload()
            # while state!=0b1:
                # pickup_payload()
            # return_to_base()
            # while state!=0b111:
                # return_to_base()
            # completed = True
            if completed:
                print("Mission complete.")
            else:
                print("Mission incomplete.")
            ended = True
        ended = True
        print("Ending.")
        sleep(1)
        print(strftime("%Y-%m-%d %H:%M:%S"))
    except KeyboardInterrupt:
        ended = True
        sleep(1)
        print(strftime("%Y-%m-%d %H:%M:%S"))
        print("Exited.")
    except Exception as e:
        ended = True
        sleep(1)
        print(strftime("%Y-%m-%d %H:%M:%S"))
        print(e)
        print("Exited.")

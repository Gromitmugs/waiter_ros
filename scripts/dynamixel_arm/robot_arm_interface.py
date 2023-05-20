#!/usr/bin/env python3

import rospy
from dynamixel_sdk import *
from waiter_ros.srv import *
import time
import csv
from datetime import datetime

# Variables
PROTOCOL_VERSION            = 2.0
DEVICENAME                  = '/dev/ttyACM0'

# Control Table Addresses
ADDR_TORQUE_ENABLE      = 64
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132
ADDR_PROFILE_VELOCITY = 112
ADDR_OPERATING_MODE = 11
ADDR_GOAL_VELOCITY = 104
ADDR_GOAL_PWM = 100
ADDR_PRESENT_INPUT_VOLTAGE = 144
ADDR_PRESENT_CURRENT = 126


BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold
ADDR_LED_RED                = 65
LEN_LED_RED                 = 1         # Data Byte Length
LEN_GOAL_POSITION           = 4         # Data Byte Length
LEN_PRESENT_POSITION        = 4         # Data Byte Length
DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
DXL_PWM_MODE = 16


# Robot Arm IDs
DXL_BASE = 1
DXL_SHOULDER_2 = 2
DXL_SHOULDER_3 = 3
DXL_ELBOW = 4
DXL_WRIST = 5
DXL_GRIPPER = 6

DXL_JOINTS = [DXL_BASE, DXL_SHOULDER_2, DXL_SHOULDER_3, DXL_ELBOW, DXL_WRIST, DXL_GRIPPER]

# Defining the limit range
# value = (min_position, max_position)
DXL_BASE_LIMIT = (0, 4095)
DXL_SHOULDER_2_LIMIT = (1600,3400)
DXL_SHOULDER_3_LIMIT = (0,4095)
DXL_ELBOW_LIMIT = (800,2500)
DXL_WRIST_LIMIT = (750,1470)
DXL_GRIPPER_LIMIT = (2024,3150)

DXL_JOINTS_LIMITS = [DXL_BASE_LIMIT, DXL_SHOULDER_2_LIMIT, DXL_SHOULDER_3_LIMIT, DXL_ELBOW_LIMIT, DXL_WRIST_LIMIT, DXL_GRIPPER_LIMIT]

DEFAULT_PROFILE_VELOCITY = 20 # value * 0.229 rpm
DEFAULT_PWM_VAL = 885

COUNT = 0

# Setup
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)
groupBulkRead = GroupBulkRead(portHandler, packetHandler)


def write_to_csv(input2dlist, name):
    with open("results/"+name+".csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerows(input2dlist)

def get_current_position(dxl_id):
    err = None
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        err = ("%s DXL_ID#%d" % (packetHandler.getTxRxResult(dxl_comm_result), dxl_id))
        return -1, err
    elif dxl_error != 0:
        err = ("%s DXL_ID#%d" % (packetHandler.getRxPacketError(dxl_error), dxl_id))
        return -1, err

    return dxl_present_position, err

def get_present_current(dxl_id):
    err = None
    dxl_present_present_current, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_CURRENT)
    if dxl_comm_result != COMM_SUCCESS:
        err = ("%s DXL_ID#%d" % (packetHandler.getTxRxResult(dxl_comm_result), dxl_id))
        return -1, err
    elif dxl_error != 0:
        err = ("%s DXL_ID#%d" % (packetHandler.getRxPacketError(dxl_error), dxl_id))
        return -1, err

    return dxl_present_present_current, err

def get_arm_current():
    arm_currents = [0,0,0,0,0,0]

    for dxl_id in DXL_JOINTS:
        dxl_present_position, err = get_present_current(dxl_id)
        if err != None:
            print(err)
        else:
            arm_currents[dxl_id-1] = dxl_present_position

    return arm_currents

def get_input_voltage(dxl_id):
    err = None
    dxl_present_input_voltage, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_INPUT_VOLTAGE)
    if dxl_comm_result != COMM_SUCCESS:
        err = ("%s DXL_ID#%d" % (packetHandler.getTxRxResult(dxl_comm_result), dxl_id))
        return -1, err
    elif dxl_error != 0:
        err = ("%s DXL_ID#%d" % (packetHandler.getRxPacketError(dxl_error), dxl_id))
        return -1, err

    return dxl_present_input_voltage, err

def gripper_control(req):
    operations = {
        0: 'close',
        1: 'open',
    }
    gripper_positions = {
        'close': 2350,
        'open': 3000,
    }

    gripper_op = operations[req.operation]
    print("gripper"+gripper_op)
    
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_GRIPPER, ADDR_GOAL_POSITION, gripper_positions[gripper_op])
    if dxl_comm_result != COMM_SUCCESS:
        print("%s DXL_ID#%d" % packetHandler.getTxRxResult(dxl_comm_result), DXL_GRIPPER)
        return False
    elif dxl_error != 0:
        print("%s DXL_ID#%d" % packetHandler.getRxPacketError(dxl_error), DXL_GRIPPER)
        return False 

    return True

def set_profile_velocity(dxl_id, vel_raw):
    err = None
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_PROFILE_VELOCITY, vel_raw)
    if dxl_comm_result != COMM_SUCCESS:
        err = ("%s DXL_ID#%d" % (packetHandler.getTxRxResult(dxl_comm_result), dxl_id))
        return err
    elif dxl_error != 0:
        err = ("%s DXL_ID#%d" % (packetHandler.getRxPacketError(dxl_error), dxl_id))
        return err

    print("Dynamixel#%d speed set to %d" % (dxl_id, vel_raw))
    return err

def initialize_dxls(DXL_JOINTS):
    for dxl_id in DXL_JOINTS:
        enable_torque(dxl_id)
        set_profile_velocity(dxl_id, DEFAULT_PROFILE_VELOCITY)

        #add param test
        dxl_addparam_result = groupBulkRead.addParam(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkRead addparam failed" % dxl_id)
            quit()

def enable_torque(dxl_id):
    err = None
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        err = ("%s DXL_ID#%d" % (packetHandler.getTxRxResult(dxl_comm_result), dxl_id))
        return err
    elif dxl_error != 0:
        err = ("%s DXL_ID#%d" % (packetHandler.getRxPacketError(dxl_error), dxl_id))
        return err

    print("Dynamixel#%d torque enabled" % dxl_id)
    return err

def disable_torque(dxl_id):
    err = None
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        err = ("%s DXL_ID#%d" % (packetHandler.getTxRxResult(dxl_comm_result), dxl_id))
        return err
    elif dxl_error != 0:
        err = ("%s DXL_ID#%d" % (packetHandler.getRxPacketError(dxl_error), dxl_id))
        return err

    print("Dynamixel#%d torque disabled" % dxl_id)
    return err

def set_param_goal_position(position):
    return [DXL_LOBYTE(DXL_LOWORD(position)),
            DXL_HIBYTE(DXL_LOWORD(position)),
            DXL_LOBYTE(DXL_HIWORD(position)),
            DXL_HIBYTE(DXL_HIWORD(position))]

def find_arm_positions():
    arm_position = [-1,-1,-1,-1,-1,-1]

    for dxl_id in DXL_JOINTS:
        dxl_present_position, err = get_current_position(dxl_id)
        if err != None:
            print(err)
        else:
            arm_position[dxl_id-1] = dxl_present_position

    return arm_position

def get_arm_position(req):
    arm_position = find_arm_positions()
    return arm_position[0], arm_position[1], arm_position[2], arm_position[3], arm_position[4], arm_position[5]

def check_base_position_in_sync(goal_position_2, goal_position_3):
    if goal_position_3 == -1:
        return False

    dxl_present_position_2, err = get_current_position(DXL_SHOULDER_2)
    if err != None:
        print(err)
        return False
    difference_2 = dxl_present_position_2 - goal_position_2


    dxl_present_position_3, err = get_current_position(DXL_SHOULDER_3)
    if err != None:
        print(err)
        return False
    difference_3 = dxl_present_position_3 - goal_position_3

    if abs(abs(difference_2) - abs(difference_3)) < 50:
        if (difference_2 < 0 and difference_3 > 0) or (difference_2 > 0 and difference_3 < 0):
            return True
    else:
        print("Base not in sync ", abs(abs(difference_2) - abs(difference_3)))
        return False

def calculate_goal_position_3(goal_position_2):
    if goal_position_2 == -1:
        return -1

    dxl_present_position_2, err = get_current_position(DXL_SHOULDER_2)
    if err != None:
        print(err)
        return -2 # -2 means to stop and exit set_arm_position
    difference = dxl_present_position_2 - goal_position_2

    dxl_present_position_3, err = get_current_position(DXL_SHOULDER_3)
    if err != None:
        print(err)
        return -2

    if goal_position_2 > dxl_present_position_2:
        print("position3 calculated -",dxl_present_position_3 - abs(difference))
        return dxl_present_position_3 - abs(difference)

    elif goal_position_2 < dxl_present_position_2:
        print("position3 calculated +",dxl_present_position_3 + abs(difference))
        return dxl_present_position_3 + abs(difference)
    return -2

def set_arm_position(req):
    global COUNT
    groupBulkWrite.clearParam()
    if req.position_3 != -1:
        print("position_3 needs to be -1 (always)")
        return False

    new_position_3 = calculate_goal_position_3(req.position_2)
    arm_positions = [req.position_1, req.position_2, new_position_3, req.position_4, req.position_5, req.position_6]

    print("position2:",req.position_2,"position_3",new_position_3)

    if req.position_2 != -1 and new_position_3 != -1:
        if not check_base_position_in_sync(req.position_2, new_position_3):
            print("Base not in sync")
            return False

    ## check
    for position, dxl_joint_limit in zip(arm_positions, DXL_JOINTS_LIMITS):
        if not (-2 <= position <= 4095):
            print("out of min-max range")
            return False

        if not (position == -1 or position == -2):
            if not (dxl_joint_limit[0] <= position <= dxl_joint_limit[1]):
                print("arm out of setting range")
                return False

        print(position)

    ## write
    exp_result = []
    for position, dxl_id in zip(arm_positions,DXL_JOINTS):
        if dxl_id == DXL_GRIPPER:
            continue
        if position == -1 or position == 0:
            continue
        if position == -2:
            return False
        
        param_goal_position = set_param_goal_position(position)
        dxl_addparam_result = groupBulkWrite.addParam(dxl_id, ADDR_GOAL_POSITION, LEN_GOAL_POSITION, param_goal_position)
        print("ID%d: position=%d" % (dxl_id, position))
        if dxl_addparam_result != True:
            print("MOVING [ID:%03d] groupBulkWrite addparam failed" % dxl_id)

    dxl_comm_result = groupBulkWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        return False
    
    while True:
        info = get_arm_current()
        info.append(datetime.now())
        exp_result.append(info)
        if check_goal_position_reached(find_arm_positions(), arm_positions):
            break

    write_to_csv(exp_result, "refill2servecurrent10TNUT"+str(COUNT))
    COUNT = COUNT + 1
    return True

def check_goal_position_reached(actual, goal):
    diffs = []
    for a,b in zip(actual,goal):
        if b <= 0:
            diffs.append(0)
        else:   
            diffs.append(abs(a) - abs(b))
    
    diffs[5] = 0 #exclude gripper
    # print(diffs)
    for diff in diffs:
        if abs(diff) > 100:
            return False
    print("goal reached diff:",diffs)
    return True

def set_torque(req):
    torque_settings = [req.torque_1, req.torque_2, req.torque_3, req.torque_4, req.torque_5, req.torque_6]
    for val in torque_settings:
        if not (0<=val<=1):
            return False

    for val, dxl_id in zip(torque_settings, DXL_JOINTS):
        if val == TORQUE_DISABLE:
            err = disable_torque(dxl_id)
            if err != None:
                print(err)
        elif val == TORQUE_ENABLE:
            err = enable_torque(dxl_id)
            if err != None:
                print(err)
    return True

def get_input_voltage_bulk(req):
    result = [-1, -1, -1, -1, -1 , -1]
    for dxl_id in DXL_JOINTS:
        dxl_present_voltage, err = get_input_voltage(dxl_id)
        if err != None:
            print(err)
        else:
            result[dxl_id-1] = dxl_present_voltage
    return result[0], result[1], result[2], result[3], result[4], result[5]

def get_input_voltage_arm():
    result = [-1, -1, -1, -1, -1 , -1]
    for dxl_id in DXL_JOINTS:
        dxl_present_voltage, err = get_input_voltage(dxl_id)
        if err != None:
            print(err)
        else:
            result[dxl_id-1] = dxl_present_voltage
    return result
    
def read_write_py_node():
    rospy.init_node('read_write_py_node')
    rospy.Service('get_arm_position', GetArmPosition, get_arm_position)
    rospy.Service('set_arm_position', SetArmPosition, set_arm_position)
    rospy.Service('set_torque', SetTorque, set_torque)
    rospy.Service('gripper_control', GripperControl, gripper_control)
    rospy.Service('get_input_voltage', GetInputVoltage, get_input_voltage_bulk)
    rospy.spin()

def main():
    try:
       portHandler.openPort()
       print("Succeeded to open the port")
    except:
        print("Failed to open the port")
        print("Press any key to terminate...")
        quit()

    try:
        portHandler.setBaudRate(BAUDRATE)
        print("Succeeded to change the baudrate")
    except:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        quit()

    initialize_dxls(DXL_JOINTS)

    print("Ready to get & set Position.")
    read_write_py_node()


if __name__ == '__main__':
    main()
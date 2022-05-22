from multiprocessing import Process, Queue, current_process
from xmlrpc.client import boolean
from matplotlib.pyplot import get
import roboticstoolbox as rtb
import time
usleep = lambda x: time.sleep(x/1000000.0)
import os
import numpy as np
from math import pi
import matplotlib as mpl
import roboticstoolbox as rtb
from queue import Queue 
import threading


import socket
import sys
import struct

from PCANBasic import *


from numpy import uint16, uint8, int16, int8
RB5_ip = "10.0.2.7"

can_packet_id = {
'FORCE_TORQUE_DATA':3,
'VOLTAGE_TEMP_STATUS_DATA':2,
'BOARD_INFO':1
}

comm_set_custom = { 
'COMM_SET_DUTY':5,
'COMM_SET_CURRENT':6,
'COMM_SET_CURRENT_BRAKE':7,
'COMM_SET_RELEASE':100,
'COMM_SET_DPS':101,
'COMM_SET_DPS_VMAX':102,
'COMM_SET_DPS_AMAX':103,
'COMM_SET_SERVO':104,
'COMM_SET_TRAJ':105
}

global pc1
global data_socket
global cmd_socket
global bPause
bPause = False

global robot
global robot_state
robot_state = -1
global iRequestSkipUpdate
iRequestSkipUpdate = 0

global MAX_SKIP_UPDATE 
MAX_SKIP_UPDATE = 10

global robot_pose
global robot_jnt
global robot_twist_pos
global robot_twist_jnt
global robot_end_rotmat
global end2ext_ft_rotmat
global ext_ft_rotmat
robot_end_rotmat = np.array([[1,0,0],
                        [0,1,0],
                        [0,0,1]])
end2ext_ft_rotmat = np.array([[0,-1,0],
                        [0,0,-1],
                        [1,0,0]])

ext_ft_rotmat = np.array([[1,0,0],
                        [0,1,0],
                        [0,0,1]])
robot_pose = np.zeros(6)
robot_jnt = np.zeros(6)
robot_twist_pose = np.zeros(6)
robot_twist_jnt = np.zeros(6)

desired_robot_twist_pose = np.zeros(6)

global step_time

# step_time = 0.005 #ms










def pause():

    bPause = True

def resume():

    bPause = False



time_prev = 0
msg_tx_type = {'MX_MY_FZ':[0xFF,0x00,00,00,00,00,00,00],
            'FX_FY_MZ':[0xFF,0x01,00,00,00,00,00,00], 
            'VOLT_ABC':[0xFF,0x02,00,00,00,00,00,00],
            'VOLT_DEF':[0xFF,0x03,00,00,00,00,00,00], 
            'INC_TEMP':[0xFF,0x04,00,00,00,00,00,00]}

msg_rx_type = {'0x0':'PCAN_MESSAGE_STANDARD',
            '0x00':'PCAN_MESSAGE_STANDARD', 
            '0x1':'PCAN_MESSAGE_RTR',
            '0x01':'PCAN_MESSAGE_RTR', 
            '0x2':'PCAN_MESSAGE_EXTENDED',
            '0x02':'PCAN_MESSAGE_EXTENDED', 
            '0x4':'PCAN_MESSAGE_FD',
            '0x04':'PCAN_MESSAGE_FD',
            '0x8':'PCAN_MESSAGE_BRS',
            '0x08':'PCAN_MESSAGE_BRS',
            '0x10':'PCAN_MESSAGE_ESI',
            '0x40':'PCAN_MESSAGE_ERRFRAME',
            '0x80':'PCAN_MESSAGE_STATUS'}


mass_arm = [
    [1, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0],
    [0, 0, 0,.2, 0, 0],
    [0, 0, 0, 0,.1, 0],
    [0, 0, 0, 0, 0,.1]
            ]


# mass_arm = [
#     [1, 0, 0, 0, 0, 0],
#     [0, 1, 0, 0, 0, 0],
#     [0, 0, 1, 0, 0, 0],
#     [0, 0, 0, 10000, 0, 0],
#     [0, 0, 0, 0, 10000, 0],
#     [0, 0, 0, 0, 0, 10000]
#             ]


# mass_arm = [
#     [10000, 0, 0, 0, 0, 0],
#     [0, 10000, 0, 0, 0, 0],
#     [0, 0, 10000, 0, 0, 0],
#     [0, 0, 0, .1, 0, 0],
#     [0, 0, 0, 0, .1, 0],
#     [0, 0, 0, 0, 0, .1]
#             ]


damping_coupling = [
    [20,0, 0, 0, 0, 0],
    [0,20, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0,10]
                   ]

damping_arm = [
    [60, 0, 0, 0, 0, 0],
    [ 0,60, 0, 0, 0, 0],
    [ 0, 0,60, 0, 0, 0],
    [ 0, 0, 0,15, 0, 0],
    [ 0, 0, 0, 0,15, 0],
    [ 0, 0, 0, 0, 0,15]
    ]

stiffness_coupling = [
    [0,0,0, 0, 0, 0],
    [0,0,0, 0, 0, 0],
    [0,0,0, 0, 0, 0],
    [0,0,0,10, 0, 0],
    [0,0,0, 0,10, 0],
    [0,0,0, 0, 0,10]
    ]

M_a_=np.array(mass_arm)*.05
D_a_=np.array(damping_arm)*0
arm_max_vel_ = 1.5
arm_max_acc_ = 1.0
platform_max_vel_ = 0.1
platform_max_acc_ = 0.2
step_time = 0.005

def pcan_Open():
    result = pc1.Initialize(PCAN_USBBUS1, PCAN_BAUD_1M)
    if result != PCAN_ERROR_OK:
        # An error occurred, get a text describing the error and show it
        #
        result = pc1.GetErrorText(result)
        print(result[1])
    else:
        print("PCAN-USB (Ch-1) was initialized")

def pcan_Close():
    # The USB Channel is released
    #
    result = pc1.Uninitialize(PCAN_USBBUS1)
    if result != PCAN_ERROR_OK:
        # An error occurred, get a text describing the error and show it
        #
        result = pc1.GetErrorText(result)
        print(result[1])
    else:
        print("PCAN-USB (Ch-1) was released")

def ReadMessage():
        # We execute the "Read" function of the PCANBasic
        #
        result = pc1.Read(PCAN_USBBUS1)
        # print(result)
        if result[0] == PCAN_ERROR_OK:
            # We show the received message
            #
            res=ProcessMessage(result[1:])
            return res
        # else:
        #     return result[0]

def ReadMessages():
    result = PCAN_ERROR_OK,
    while (result[0] & PCAN_ERROR_QRCVEMPTY) != PCAN_ERROR_QRCVEMPTY:
        result = pc1.Read(PCAN_USBBUS1)
        
        if result[0] != PCAN_ERROR_QRCVEMPTY:
            res = ProcessMessage(result[1:])
            return res
        else:
            if(result[0] != PCAN_ERROR_QRCVEMPTY):  print('ERROR_CODE:{}'.format(hex(result[0])))
            # return result[0]
    
            

def ProcessMessage(*args):
    global time_prev
    
    
    theMsg = args[0][0]
    itsTimeStamp = args[0][1]    

    newMsg = TPCANMsgFD()
    newMsg.ID = theMsg.ID
    newMsg.DLC = theMsg.LEN
    for i in range(8 if (theMsg.LEN > 8) else theMsg.LEN):
        newMsg.DATA[i] = theMsg.DATA[i]
    newMsg.MSGTYPE = theMsg.MSGTYPE
    newTimestamp = TPCANTimestampFD()
    newTimestamp.value = (itsTimeStamp.micros + 1000 * itsTimeStamp.millis + 0x100000000 * 1000 * itsTimeStamp.millis_overflow)

    time = "Timestamp:{:0.3f}sec".format(newTimestamp.value/1000000)
    period = newTimestamp.value - time_prev
    cycle_time = "Cycle Time:{:0.3f}msec".format(period/1000)
    TYPE = "TYPE:{}".format(msg_rx_type[hex(newMsg.MSGTYPE)])
    EID = "EID:{}".format(hex(newMsg.ID))
    DLC = "DLC:{}".format(newMsg.DLC)
    DATA = ' '.join('{:02x}'.format(newMsg.DATA[i]) for i in range(newMsg.DLC))
    
    # if newMsg.MSGTYPE == 0x00:  # PCAN_MESSAGE_STANDARD 
        # print(time,"|",TYPE,"|",EID,"|",DLC,"|",DATA,"|",cycle_time)
    time_prev = newTimestamp.value
    return newMsg.DATA

def WriteFrame(id, dlc, data):   
    CANMsg = TPCANMsg()

    CANMsg.ID = id
    CANMsg.LEN = dlc
    CANMsg.MSGTYPE = PCAN_MESSAGE_STANDARD

    for i in range(CANMsg.LEN):
        CANMsg.DATA[i] = data[i]

    return pc1.Write(PCAN_USBBUS1, CANMsg)

def GetFormatedError(error):
    stsReturn = pc1.GetErrorText(error, 0)
    if stsReturn[0] != PCAN_ERROR_OK:
        return "An error occurred. Error-code's text ({0:X}h) couldn't be retrieved".format(error)
    else:
        return stsReturn[1]

def read_ft_data():
    Fx=0
    Fy=0
    Fz=0
    Mx=0
    My=0
    Mz=0
    WriteFrame(can_packet_id['FORCE_TORQUE_DATA'],8,msg_tx_type['FX_FY_MZ'])
    res_flg=0
    while res_flg!=2:
        result=ReadMessages()
        if result !=None:
            if result[0]==1:
                    Mx = 0.01*int16(result[2]<<8|result[1])
                    My = 0.01*int16(result[4]<<8|result[3])
                    Fz =  0.1*int16(result[6]<<8|result[5])
                    # Mx = 0.01*int16(result[2]>>8|result[1])
                    # My = 0.01*int16(result[4]>>8|result[3])
                    # Fz =  0.1*int16(result[6]>>8|result[5])
                    res_flg=1
            elif result[0]==2:
                Fx = 0.1*int16(result[2]<<8|result[1])
                Fy = 0.1*int16(result[4]<<8|result[3])
                Mz = 0.01*int16(result[6]<<8|result[5])
                # Fx = 0.1*int16(result[2]>>8|result[1])
                # Fy = 0.1*int16(result[4]>>8|result[3])
                # Mz = 0.01*int16(result[6]>>8|result[5])
                res_flg=2
        
        res=np.array([Fx, Fy, Fz, Mx, My, Mz])
    # print("Fx= {:.4f}, Fy= {:.4f}, Fz= {:.4f}, Mx= {:.4f}, My= {:.4f}, Mz= {:.4f}".format(Fx, Fy, Fz, Mx, My, Mz))
    return res

def calib_ft_data():
    ft_data_ = np.zeros(6)
    ft_calib_= np.zeros(6)
    for i in range(100):
        ft_data_ += read_ft_data()
        
    # for i in range(len(ft_data_)):
    #     ft_calib_[i]=ft_data_[i]/100
    ft_calib_=ft_data_/100
    print(ft_calib_)
    return ft_calib_


def Rb5_decode(data):
    decode_data=[]
    for i in range(26):
        data_4byte=struct.unpack('f',data[i*4:i*4+4])[0]   
        # print(data_4byte)
        decode_data.append(data_4byte)
    return decode_data

def Rb5_getdata(sock):
    bufsize = 4096
    msg='reqdata'
    sdata=msg.encode()
    sock.send(sdata)
    data = sock.recv(bufsize)
    res=Rb5_decode(data)
        
    # print(res)
    return res
            
def get_data_thread(sock, q_in, q_out):
    print('start')
    ft_calib=calib_ft_data()
    while True:
        cmd=q_in.get()
        if cmd == 1:
            rb5_data_ = Rb5_getdata(sock)
            ft_data_ =  read_ft_data()-ft_calib
            q_out.put(ft_data_)
            q_out.put(rb5_data_)
            
def sendCommand(cmd):
    global cmd_socket
    # print(cmd)
    send_cmd=cmd.encode()

    # while(bPause == true):
    
    #     usleep(1000*100)
    
    cmd_socket.send(send_cmd)
    # print("send_command")


def isMotionIdle():
    global robot_state
    global iRequestSkipUpdate
    print(iRequestSkipUpdate)
    if(iRequestSkipUpdate >0):
        
    #    //     printf("IsMotionIdle : false\r\n");
        return False
        
        # //else if robot is idle, finish waiting
    #   //  printf("IsMotionIdle : %d\r\n",(systemStat.sdata.robot_state == 1) || (systemStat.sdata.robot_state == 0) );
    return ((robot_state == 1) or (robot_state == 0))


def waitUntilMotionDone():
    usleep(10)
    # while(isMotionIdle() != True):
    #     print(isMotionIdle())
    #     usleep(10)


def initRobot():

    sendCommand("mc jall init")
    time.sleep(1)


#  <ProgramMode_Real>
#  change to 'real robot' mode -- robot will move
def setRealMode():

    sendCommand("pgmode real")
    time.sleep(1)
    # return *this;

# <ProgramMode_Simulation>
# change to 'simulation' mode -- robot will not move except teaching
def setSimulationMode():

    sendCommand("pgmode simulation")
    time.sleep(1)
    # return *this;


# // <MoveJoint>
# // : move to target posture in joint coordinate
# // joint1~joint6 : target joint angle in deg unit
# // spd : speed parameter (0~1: user define   or   -1: default setting)
# // acc : acceleration parameter (0~1: user define   or   -1: default setting)
def moveJoint(joint, spd = -1.0, acc = -1.0):
    return moveJoint_raw(joint[0],joint[1],joint[2],joint[3],joint[4],joint[5],spd,acc)

def moveJoint_raw(joint1, joint2, joint3, joint4, joint5, joint6, spd=-1.0, acc=-1.0):
    global robot_state 
    global iRequestSkipUpdate
    sendCommand("jointall {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}".format(spd,
                acc,
                joint1,
                joint2,
                joint3,
                joint4,
                joint5,
                joint6))
    robot_state = 3 #run
    iRequestSkipUpdate = MAX_SKIP_UPDATE
    waitUntilMotionDone()
    


    # // If you send "move_servo_j(jnt[ANGLE0, ANGLE1, ANGLE2, ANGLE3, ANGLE4, ANGLE5], time1, time2, gain, lpf_gain)"
    # // to our system through port number 5000, you can move joints in soft-real-time.
    # // time1 : user's command sending period/gap (unit: second) (>0.002) : default = 0.002
    # // time2 : lookahead time (unit: second) (0.021 ~ 0.2) : default = 0.1
    # // gain : control gain (0~1) : default = 0.02
    # // lpf_gain : low pass filter gain (0~1) : default = 0.2
    # // This function is the same as UR's servo_j function.


def servoJoint(joint, time1 = 0.002, time2 = 0.1, gain = 0.02, lpf_gain=0.2):
 
    return servoJoint_raw(joint[0],joint[1],joint[2],joint[3],joint[4],joint[5],time1, time2, gain, lpf_gain)
 
def servoJoint_raw(joint1, joint2, joint3, joint4, joint5, joint6, time1 = 0.002, time2 = 0.1, gain = 0.02, lpf_gain=0.2):
    global robot_state 
    global iRequestSkipUpdate

    sendCommand("move_servo_j(jnt[{:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}],{:.3f}, {:.3f}, {:.3f}, {:.3f})".format(
                joint1,
                joint2,
                joint3,
                joint4,
                joint5,
                joint6,
                time1,  
                time2, 
                gain, 
                lpf_gain))
    robot_state = 3 #run
    iRequestSkipUpdate = MAX_SKIP_UPDATE
    waitUntilMotionDone()
    



# // <MoveTCP>
# // : move to target posture in cartesian coordinate
# // x, y, z : target TCP(tool center point) position in mm unit
# // rx, ry, rz : target TCP orientation (Yaw-Pitch-Roll Euler angle) in degree unit
# // spd : speed parameter (0~1: user define   or   -1: default setting)
# // acc : acceleration parameter (0~1: user define   or   -1: default setting)
def  moveTCP(pose, spd = -1, acc=-1):

    return moveTCP_raw(pose[0],pose[1],pose[2],pose[3],pose[4],pose[5],spd,acc)

def  moveTCP_raw(x, y, z, rx, ry, rz, spd = -1, acc = -1):

    sendCommand("movetcp {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}".format(
        spd,
        acc,
        x,
        y,
        z,
        rx,
        ry,
        rz))
    robot_state = 3; #run
    iRequestSkipUpdate = MAX_SKIP_UPDATE
    waitUntilMotionDone()


def get_robot_param():
    global rb5_data
    global robot_pose
    global robot_jnt
    global robot_twist_pose 
    global robot_twist_jnt
    global step_time
    
    robot_jnt_now = np.array(rb5_data[2:8])*pi/180.0
    robot_pose_now = np.array(rb5_data[20:26])/1000.0
    robot_twist_pose = (robot_pose_now-robot_pose)/step_time
    robot_twist_jnt = (robot_jnt_now-robot_jnt)/step_time
    # print(robot_twist_pose)
    # print(robot_twist_jnt)
    
    robot_pose = robot_pose_now 
    robot_jnt = robot_jnt_now
    print(robot_pose)
    





# def compute_admittance(): 
#     global rb5_data
#     global robot_pose
#     global robot_jnt
#     global robot_twist_pose 
#     global desired_robot_twist_pose 
#     global robot_twist_jnt
#     global step_time
#     global robot
#     global ext_ft_rotmat

#     # Vector6d platform_desired_acceleration
#     wrench_external_=wrench_external_callback()
#     arm_desired_accelaration = np.zeros(6)

#     arm_desired_accelaration = np.dot(np.linalg.inv(M_a_) , ( np.dot(- D_a_,robot_twist_pose)+ wrench_external_))
#                             #    + wrench_external_ + wrench_control_)
                            
#     arm_desired_accelaration_jacob = np.dot(np.linalg.inv(robot.jacob0(robot_jnt)) ,arm_desired_accelaration)
#     #                         #    + wrench_external_ + wrench_control_)
#     # print(robot_pose)
#     # print(np.linalg.inv(robot.jacob0(robot_jnt)))
#     # limiting the accelaration for better stability and safety
#     a_acc_norm = np.linalg.norm(arm_desired_accelaration[0:3])
#     if (a_acc_norm > arm_max_acc_):
#         arm_desired_accelaration[0:3] *= (arm_max_acc_ / a_acc_norm)
    

#     # Integrate for velocity based interface
#     arm_desired_err_ = 0.5 * arm_desired_accelaration * step_time* step_time
#     arm_desired_jnt_err_ = robot_twist_jnt* step_time + 0.5 * arm_desired_accelaration_jacob * step_time* step_time
#     # print(cmd_jnt*180/pi)
#     # print(arm_desired_jnt_err_*180/pi)
#     # servoJoint((cmd_jnt-arm_desired_jnt_err_)*180/pi)
#     servoJoint((robot_jnt+arm_desired_jnt_err_)*180/pi)
    
    
    
def compute_admittance(): 
    global rb5_data
    global robot_pose
    global robot_jnt
    global robot_twist_pose 
    global desired_robot_twist_pose 
    global robot_twist_jnt
    global step_time
    global robot
    global ext_ft_rotmat

    # Vector6d platform_desired_acceleration
    wrench_external_=wrench_external_callback()
    arm_desired_accelaration = np.zeros(6)

    arm_desired_accelaration = np.dot(np.linalg.inv(M_a_) , ( - np.dot(D_a_,robot_twist_pose)+ wrench_external_))
                            #    + wrench_external_ + wrench_control_)
    # print(arm_desired_accelaration)   
    # desired_robot_twist_pose = robot_twist_pose+arm_desired_accelaration * step_time
    
    print(wrench_external_)   
    print(-np.dot(D_a_,robot_twist_pose))
    # print(robot_twist_pose)   
    print(arm_desired_accelaration)
    arm_desired_err_ = robot_twist_pose + arm_desired_accelaration * step_time
    print('arm')
    print(arm_desired_err_)
    arm_desired_jnt_err_ = np.dot(np.linalg.inv(robot.jacob0(robot_jnt)) ,arm_desired_err_)
    #                         #    + wrench_external_ + wrench_control_)
    print(arm_desired_jnt_err_)
    # print(robot_pose)
    # print(np.linalg.inv(robot.jacob0(robot_jnt)))
    # limiting the accelaration for better stability and safety
    a_acc_norm = np.linalg.norm(arm_desired_accelaration[0:3])
    if (a_acc_norm > arm_max_acc_):
        arm_desired_accelaration[0:3] *= (arm_max_acc_ / a_acc_norm)
    

    # Integrate for velocity based interface
    servoJoint((robot_jnt+arm_desired_jnt_err_* step_time)*180/pi)    
    
    

def get_rotation_matrix():
    global robot
    
    global robot_end_rotmat
    global end2ext_ft_rotmat
    global ext_ft_rotmat
    
    rot_mat = np.array(robot.fkine(robot_jnt))
    # print(rot_mat)
    robot_end_rotmat = rot_mat[:3,:3]
    ext_ft_rotmat = np.dot(robot_end_rotmat,end2ext_ft_rotmat)

def wrench_external_callback():
    
    global ext_ft_rotmat
    global ft_data
    get_rotation_matrix()
    ft_data_base = np.concatenate([np.dot(ext_ft_rotmat,ft_data[0:3]),np.dot(ext_ft_rotmat,ft_data[3:6])])   
    # print(ft_data) 
    # print(ft_data_base)
    return ft_data_base
    
# void AdmittanceController::wrench_control_callback(
#   const geometry_msgs::WrenchStampedConstPtr msg) {

#   if (msg->header.frame_id == "arm_base_link") {
#     wrench_control_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
#                     msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
#   }
#   else  {
#     ROS_WARN_THROTTLE(5, "wrench_control_callback: The frame_id is not specified as ur5_arm_base_link");
#   }
# }

    

if __name__ == "__main__":
    q_in =Queue(1)
    q_out =Queue(2)
    global ft_data
    global rb5_data
    global cmd_jnt
    global cmd_pose
    robot = rtb.models.DH.RB5()
    
    
    host = RB5_ip
    data_port = 5001
    cmd_port = 5000
    ###### Initializing #######
    if len(sys.argv)>1:
        host = sys.argv[1]

    data_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    cmd_socket  = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    pc1 = PCANBasic()
    pcan_Open()
    try:
        data_socket.connect((host, data_port))
        cmd_socket.connect((host, cmd_port))
        
        
        print("start")

        ###### Start daemon thread to get External F/T sensor data & RB5 data  #######
        t1 = threading.Thread(target=get_data_thread, args=(data_socket, q_in, q_out))
        t1.daemon = True 
        t1.start()
        print("start_tread")
        initRobot()
        print("init")
        setRealMode()
        print("set_real_mode")
        q_in.put(1)
        ft_data=q_out.get()
        rb5_data=q_out.get()
        get_robot_param()
        cmd_jnt = robot_jnt
        cmd_pose = robot_pose
        # cmd_jnt_1=cmd_jnt*180/pi + [5,5,5,5,5,5]
        # print(cmd_jnt_1)
        # servoJoint(cmd_jnt)
        # moveJoint([0,0,0,0,0,0])
        # print("jnt_move")
        while True:
            # usleep(step_time*1000)
            # usleep(5*1000)
            time.sleep(step_time)
            q_in.put(1)
            ft_data=q_out.get()
            rb5_data=q_out.get()
            get_robot_param()
            compute_admittance()
            # print(ft_data)
        print("### End ###")
        
        
        
    except KeyboardInterrupt:
        pass
    finally:
        os._exit(0) # not call close() ** workaround for native dead

    print("\nOutput:%s" % OUTFILE_NAME)
#eof
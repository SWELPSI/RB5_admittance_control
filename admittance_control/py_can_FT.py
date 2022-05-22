from turtle import end_fill, fd

from numpy import uint16, uint8, int16, int8
from PCANBasic import *


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
                    Mx = 0.01*int8(result[2]<<8|result[1])
                    My = 0.01*int8(result[4]<<8|result[3])
                    Fz =  0.1*int8(result[6]<<8|result[5])
                    res_flg=1
            elif result[0]==2:
                Fx = 0.1*int8(result[2]<<8|result[1])
                Fy = 0.1*int8(result[4]<<8|result[3])
                Mz = 0.01*int8(result[6]<<8|result[5])
                res_flg=2
        
        res=[Fx, Fy, Fz, Mx, My, Mz]
    print("Fx= {:.4f}, Fy= {:.4f}, Fz= {:.4f}, Mx= {:.4f}, My= {:.4f}, Mz= {:.4f}".format(Fx, Fy, Fz, Mx, My, Mz))
    return res


if __name__ == '__main__':
    pc1 = PCANBasic()

    # Open PCAN Device
    pcan_Open()
    read_ft_data()
    # WriteFrame(can_packet_id['FORCE_TORQUE_DATA'],8,msg_tx_type['MX_MY_FZ'])
    # ReadMessages()
    # WriteFrame(can_packet_id['FORCE_TORQUE_DATA'],8,msg_tx_type['FX_FY_MZ'])
    
    # CAN MSG Read Routine
    num = 0
    # try~ except 특정 예외
    try:
        # 무한 반복
        while True:
            read_ft_data()
            # print(result)
            num += 1
    # Ctrl + C를 입력할 경우
    except KeyboardInterrupt:
        print('Total Rcv number is {}, Quit to receive'.format(num))


    # CAN MSG Write Routine
    #VescCustumControl(83, 'COMM_SET_DUTY',0.1)
    #VescCustumControl(83, 'COMM_SET_CURRENT',0.5)
    #VescCustumControl(83, 'COMM_SET_CURRENT_BRAKE',5)
    #VescCustumControl(83, 'COMM_SET_DPS',0) # position lock on current position
    #VescCustumControl(83, 'COMM_SET_DPS',1000)
    #VescCustumControl(83, 'COMM_SET_DPS_VMAX',2000)
    #VescCustumControl(83, 'COMM_SET_SERVO',0)
    # VescCustumControl(83, 'COMM_SET_RELEASE',0)

    # Close PCAN Device
    pcan_Close()
import dynamixel_sdk
import time 
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel
DEVICENAME                  = '/dev/ttyUSB0'          # Check which port is being used on your controller

BAUDRATE1                    = 57600             # HEAD Baudrate 

# Control table address
ADDR_MX_GOAL_POSITION      = 30
#ADDR_MX_PRESENT_POSITION   = 36

ID_EYES_UD = 6
ID_EYES_LR = 7

EYES_UD_MIN = 1600
EYES_UD_MAX = 2600

EYES_LR_MIN = 1400
EYES_LR_MAX = 2600



def open_port(BAUDRATE):
    portHandler = dynamixel_sdk.PortHandler(DEVICENAME)
    packetHandler = dynamixel_sdk.PacketHandler(PROTOCOL_VERSION)
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        quit()
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        quit()
    return portHandler, packetHandler


def set_position(dxl_id, goal_position, packetHandler, portHandler):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, dxl_id, ADDR_MX_GOAL_POSITION, goal_position)
    if dxl_comm_result != dynamixel_sdk.COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

PORT_H1, PACKET_H1 = open_port(BAUDRATE1)

def main():

    start_j = EYES_UD_MIN

    i = EYES_LR_MIN
    while i < EYES_LR_MAX:

        set_position(ID_EYES_LR, i, PACKET_H1, PORT_H1)

        if start_j == EYES_UD_MIN:
            j = start_j
            while j < EYES_UD_MAX:
                time.sleep(0.020)
                set_position(ID_EYES_UD, j, PACKET_H1, PORT_H1)
                j += 10
            start_j = EYES_UD_MAX

        else:
            j = start_j
            while j > EYES_UD_MIN:
                time.sleep(0.020)
                set_position(ID_EYES_UD, j, PACKET_H1, PORT_H1)
                j -= 10
            start_j = EYES_UD_MIN

        i += 300

main()

    


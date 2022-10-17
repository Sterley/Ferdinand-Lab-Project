import dynamixel_sdk
from sympy import false, true

PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel
DEVICENAME                  = '/dev/ttyUSB0'          # Check which port is being used on your controller

BAUDRATE1                    = 57600             # HEAD Baudrate
BAUDRATE2                    = 1000000           # ELBOW Baudrate     

# Control table address
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36

LEFT_EYEBROW  = 5
RIGHT_EYEBROW  = 4
RIGHT_SHOULDER = 20
RIGHT_ELBOW  = 23


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

def get_position(dxl_id, packetHandler, portHandler):
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, dxl_id, ADDR_MX_PRESENT_POSITION)
    if dxl_comm_result != dynamixel_sdk.COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    return  dxl_present_position


def eyebrow_down():
    portH1, packetH1 = open_port(BAUDRATE1)
    set_position(LEFT_EYEBROW,2080, packetH1, portH1)
    set_position(RIGHT_EYEBROW,1830, packetH1, portH1)


def eyebrow_up():
    portH1, packetH1 = open_port(BAUDRATE1)
    set_position(LEFT_EYEBROW, 2230, packetH1, portH1)
    set_position(RIGHT_EYEBROW,1680, packetH1, portH1)


def have_to_go():
    tmp = false
    portH2, packetH2 = open_port(BAUDRATE2)
    pos_should = get_position(RIGHT_SHOULDER, packetH2, portH2)
    pos_elbow = get_position(RIGHT_ELBOW, packetH2, portH2)
    if pos_should < 2800 or pos_elbow > 2700:
        tmp = true
    return tmp

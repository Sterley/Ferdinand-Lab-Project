#!/usr/bin/env python3
import rospy

# To control the different motors of the robot
import dynamixel_sdk

# to control the time to send orders
import time 

# Message Object Format to receive face detection datas
from ferdinand_msgs.msg import FaceDetectionData

# See which protocol version is used in the Dynamixel
PROTOCOL_VERSION            = 1.0               

# Check which port is being used on your controller
DEVICENAME                  = '/dev/ttyUSB0'          

# Reachy's Head Baudrate
BAUDRATE1                    = 57600     

# Reachy's trunk Baudrate         
BAUDRATE2                    = 1000000  

# Control table address
ADDR_MX_GOAL_POSITION        = 30
ADDR_MX_PRESENT_POSITION     = 36
ADDR_MX_TORQUE_ENABLE        = 24

# Eyes Up Down Motor ID
ID_EYES_UD = 6
# Eyes Left Right Motor ID
ID_EYES_LR = 7
# Neck Up Down Motor ID
ID_NECK_UD = 3
# Necks Left Right Motor ID
ID_NECK_P_LR = 2
ID_NECK_LR = 1

# Eyes Up Down Motor Min Position
EYES_UD_MIN = 1600
# Eyes Up Down Motor Max Position
EYES_UD_MAX = 2600

# Eyes Left Right Motor Min Position
EYES_LR_MIN = 1400
# Eyes Left Right Motor Max Position
EYES_LR_MAX = 2500

# Neck Left Right Motor Min Position
NECK_LR_MIN = 1700
# Neck Left Right Motor Max Position
NECK_LR_MAX = 2400

# Eyes Left Right Motor Initial Position
POS_LR = 1800
# Neck Left Right Motor Initial Position
POS_LR_N = 2050
# Eyes Ud Down Motor Initial Position
POS_UD = 2100

# Counter that calculates the number of frames for which data has been received
CP_DATA_FLOW_INIT = 0
# The number of frames used to average the data for decision making on the robot
CP_DATA_FLOW_MAX = 3

# The proportionality coefficient of the pid to move the eyes
Kp = 0.5
# The proportionality coefficient of the pid to move the neck
Kp2 = 0.05
# The derivative coefficient of the pid to move the eyes
Kd = 0.1
# the previous error for the pid for the eyes up down motor
erreur_lr_pred = 0
# the previous error for the pid for the eyes left right motor
erreur_ud_pred = 0

# The sum of xmin's for the data's average for decision making on the robot
SUM_XMIN = 0
# The sum of ymin's for the data's average for decision making on the robot
SUM_YMIN = 0
# The sum of xmax's for the data's average for decision making on the robot
SUM_XMAX = 0
# The sum of ymax's for the data's average for decision making on the robot
SUM_YMAX = 0

# Motor ID for left eyebrows
LEFT_EYEBROW  = 5
# Motor ID for right eyebrows
RIGHT_EYEBROW  = 4
# Motor ID for the right should
RIGHT_SHOULDER = 20
# Motor ID for the right elbow
RIGHT_ELBOW  = 23

# Opening the port for sending commands to the robot motors at a certain BAUDRATE
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

# To set a motor to a certain position
def set_position(dxl_id, goal_position, packetHandler, portHandler):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, dxl_id, ADDR_MX_GOAL_POSITION, goal_position)
    if dxl_comm_result != dynamixel_sdk.COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

# To get the position of a motor
def get_position(dxl_id, packetHandler, portHandler):
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, dxl_id, ADDR_MX_PRESENT_POSITION)
    if dxl_comm_result != dynamixel_sdk.COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    return  dxl_present_position

# To fix or defix a motor
def set_torque(dxl_id, torque_value, packetHandler, portHandler):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, torque_value)
    if dxl_comm_result != dynamixel_sdk.COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        pass
        


# To look further to the right
# command is the step size calculated with the PID
def go_right(erreur):
    global POS_LR 
    global POS_LR_N
    
    global Kp
    global Kp2
    global Kd
    global erreur_lr_pred
    commande = Kp * erreur + Kd * (erreur - erreur_lr_pred)
    erreur_lr_pred = erreur
    
    POS_LR -= int(commande)
    if POS_LR < EYES_LR_MIN:
        POS_LR = EYES_LR_MIN
    set_position(ID_EYES_LR, POS_LR, PACKET_H1, PORT_H1)
    
    if POS_LR < 1800:
        erreur = 1800-POS_LR
        commande = Kp2 * erreur 
        POS_LR_N += int(commande)
        if POS_LR_N > NECK_LR_MAX:
            POS_LR_N = NECK_LR_MAX
        set_position(ID_NECK_LR, POS_LR_N, PACKET_H1, PORT_H1)
        

# To look further to the left
# command is the step size calculated with the PID        
def go_left(erreur):
    global POS_LR
    global POS_LR_N
    
    global Kp
    global Kp2
    global Kd
    global erreur_lr_pred
    commande = Kp * erreur + Kd * (erreur - erreur_lr_pred)
    erreur_lr_pred = erreur
    
    POS_LR += int(commande)
    if POS_LR > EYES_LR_MAX:
        POS_LR = EYES_LR_MAX
    set_position(ID_EYES_LR, POS_LR, PACKET_H1, PORT_H1)
    
    if POS_LR > 1800:
        erreur = 1800-POS_LR
        commande = Kp2 * erreur 
        POS_LR_N -= int(-commande)
        if POS_LR_N < NECK_LR_MIN:
            POS_LR_N = NECK_LR_MIN
        set_position(ID_NECK_LR, POS_LR_N, PACKET_H1, PORT_H1)

# To look further up
# command is the step size calculated with the PID  
def go_up(erreur):
    global POS_UD
    
    global Kp
    global Kd
    global erreur_ud_pred
    commande = Kp * erreur + Kd * (erreur - erreur_ud_pred)
    erreur_ud_pred = erreur
    
    POS_UD += int(commande)
    if POS_UD > EYES_UD_MAX:
        POS_UD = EYES_UD_MAX
    set_position(ID_EYES_UD, POS_UD, PACKET_H1, PORT_H1)
    
# To look further down
# command is the step size calculated with the PID  
def go_down(erreur):
    global POS_UD
    
    global Kp
    global Kd
    global erreur_ud_pred
    commande = Kp * erreur + Kd * (erreur - erreur_ud_pred)
    erreur_ud_pred = erreur
    
    POS_UD -= int(commande)
    if POS_UD < EYES_UD_MIN:
        POS_UD = EYES_UD_MIN
    set_position(ID_EYES_UD, POS_UD, PACKET_H1, PORT_H1)   

# Raise eyebrows
def eyebrow_down():
    portH1, packetH1 = open_port(BAUDRATE1)
    set_position(LEFT_EYEBROW,2080, packetH1, portH1)
    set_position(RIGHT_EYEBROW,1830, packetH1, portH1)

# Lower eyebrows
def eyebrow_up():
    portH1, packetH1 = open_port(BAUDRATE1)
    set_position(LEFT_EYEBROW, 2230, packetH1, portH1)
    set_position(RIGHT_EYEBROW,1680, packetH1, portH1)

# Detect if they are holding his arm to ask him to follow
def have_to_go():
    tmp = False
    portH2, packetH2 = open_port(BAUDRATE2)
    pos_should = get_position(RIGHT_SHOULDER, packetH2, portH2)
    pos_elbow = get_position(RIGHT_ELBOW, packetH2, portH2)
    if pos_should < 2800 or pos_elbow > 2700:
        tmp = True
    return tmp
    

# Search for a person around him to follow him   
# When someone is detected, it stops searching
def scan_env():
    global CP_DATA_FLOW_INIT
    start_j = EYES_UD_MIN

    i = EYES_LR_MIN
    while i < EYES_LR_MAX:

        set_position(ID_EYES_LR, i, PACKET_H1, PORT_H1)

        if start_j == EYES_UD_MIN:
            j = start_j
            while j < EYES_UD_MAX:
                time.sleep(0.020)
                set_position(ID_EYES_UD, j, PACKET_H1, PORT_H1)
                if CP_DATA_FLOW_INIT == 2:
                    print("Face detected !")               
                    return True
                j += 10
            start_j = EYES_UD_MAX

        else:
            j = start_j
            while j > EYES_UD_MIN:
                time.sleep(0.020)
                set_position(ID_EYES_UD, j, PACKET_H1, PORT_H1)
                if CP_DATA_FLOW_INIT == 2:
                    print("Face detected !")
                    return True
                j -= 10
            start_j = EYES_UD_MIN

        i += 300
    print("Scan_env Done ! No Face detected !")
    return False

# To track the face of the detected person
# We compute the center of gravity of the base image 
# and the center of gravity of the rectangle around the detected face 
# and compare them for decision making
def move(width, height, xmin, ymin, xmax, ymax):
    ctg_img = (width/2, height/2)
    ctg_detection = (((xmax-xmin)/2)+xmin, ((ymax-ymin)/2)+ymin) 
    erreur_lr = abs(ctg_detection[0] - ctg_img[0])
    if ctg_detection[0] < ctg_img[0]:
        go_right(erreur_lr)
    elif ctg_detection[0] > ctg_img[0]:
        go_left(erreur_lr)
    else:
        pass
    erreur_ud = abs(ctg_detection[1] - ctg_img[1])
    if ctg_detection[1] < ctg_img[1]:
        go_up(erreur_ud)
    elif ctg_detection[1] > ctg_img[1]:
        go_down(erreur_ud)
    else:
        pass


# callback function that processes data when received from the topic /face_detection_data
def callback_receive_data(msg):
    global CP_DATA_FLOW_INIT
    global CP_DATA_FLOW_MAX
    global SUM_XMIN 
    global SUM_YMIN 
    global SUM_XMAX
    global SUM_YMAX
    CP_DATA_FLOW_INIT += 1
    SUM_XMIN += msg.xmin
    SUM_YMIN += msg.ymin
    SUM_XMAX += msg.xmax
    SUM_YMAX += msg.ymax
    if CP_DATA_FLOW_INIT == CP_DATA_FLOW_MAX:
        move(msg.width, msg.height, SUM_XMIN/CP_DATA_FLOW_MAX, SUM_YMIN/CP_DATA_FLOW_MAX, SUM_XMAX/CP_DATA_FLOW_MAX, SUM_YMAX/CP_DATA_FLOW_MAX)
        CP_DATA_FLOW_INIT = 0
        SUM_XMIN = 0
        SUM_YMIN = 0
        SUM_XMAX = 0
        SUM_YMAX = 0


   
        
if __name__ == '__main__':

    # We test if we should follow a person
    TEST_HAVE_TO_GO = True
    
    # Node Initialization
    rospy.init_node('my_first_python_node')
    rospy.loginfo("This node has been started")
    
    # Port Opening
    PORT_H1, PACKET_H1 = open_port(BAUDRATE1) 
    
    # Rate Fixation 
    rate = rospy.Rate(10)
    
    # Set motors to initial position
    set_position(ID_EYES_UD, POS_UD, PACKET_H1, PORT_H1)
    set_position(ID_EYES_LR, POS_LR, PACKET_H1, PORT_H1)
    set_position(ID_NECK_LR, POS_LR_N, PACKET_H1, PORT_H1)
    set_torque(ID_NECK_UD, 1, PACKET_H1, PORT_H1)
    set_torque(ID_NECK_P_LR, 1, PACKET_H1, PORT_H1)
    
    sub = None 
    while not rospy.is_shutdown():
        if(TEST_HAVE_TO_GO):  
            # Waiting for someone to hold his hand
            if have_to_go():
                # If someone has held his arm, he raises his eyebrows, looks around
                eyebrow_up()
                print("Following Signal Successfully Received")
                sub = rospy.Subscriber("/face_detection_data", FaceDetectionData, callback_receive_data)
                print("Receiving Data From Camera Node")
                scan_env()
                TEST_HAVE_TO_GO = False
        rate.sleep()


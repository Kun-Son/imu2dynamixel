#!/usr/bin/env python

import rospy
import os
import ctypes
from beginner_tutorials.msg import Data

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

#os.sys.path.append('../dynamixel_functions_py')             # Path setting

import dynamixel_functions as dynamixel                     # Uses Dynamixel SDK library

# Control table address
ADDR_MX_TORQUE_ENABLE       = 24                            # Control table address is different in #Dynamixel model
ADDR_MX_GOAL_TORQUE         = 71
ADDR_MX_TORQUE_CONT	    = 70

# Data Byte Length
LEN_MX_GOAL_TORQUE          = 2
LEN_MX_TORQUE_CTRL          = 1

# Protocol version
PROTOCOL_VERSION            = 1                             # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                      = 1                             # Dynamixel ID: 1

BAUDRATE                    = 1000000
DEVICENAME                  = "/dev/ttyUSB0".encode('utf-8')# Check which port is being used on your controller
                                                            # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque

DXL_MOVING_STATUS_THRESHOLD = 3                            # Dynamixel moving status threshold

ESC_ASCII_VALUE             = 0x1b

COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed

port_num = dynamixel.portHandler(DEVICENAME)

# Initialize PacketHandler Structs
dynamixel.packetHandler()

# Initialize Groupsyncwrite instance
group_num = dynamixel.groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_MX_GOAL_TORQUE, LEN_MX_GOAL_TORQUE)
torque_mode_on = dynamixel.groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_MX_TORQUE_CONT, LEN_MX_TORQUE_CTRL)

dxl_comm_result = COMM_TX_FAIL                              # Communication result

def callback(data):
	rospy.loginfo(data.data)
	dxl_torcon(data.data)

def dxl_torcon(goal_torq):
	 # Write goal position
    dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, 1, ADDR_MX_GOAL_TORQUE, goal_torq[0])
    if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
        dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
    elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
        dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))

def torque_ctrl():
	dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, 1, ADDR_MX_TORQUE_CONT, 1)
	if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
	    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
	elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
	    dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))



def dxl_init():
	
	# Initialize PortHandler Structs
	# Set the port path
	# Get methods and members of PortHandlerLinux or PortHandlerWindows
	
	# Open port
	if dynamixel.openPort(port_num):
	    print("Succeeded to open the port!")
	else:
	    print("Failed to open the port!")
	    print("Press any key to terminate...")
	    getch()
	    quit()

	# Set port baudrate
	if dynamixel.setBaudRate(port_num, BAUDRATE):
	    print("Succeeded to change the baudrate!")
	else:
	    print("Failed to change the baudrate!")
	    print("Press any key to terminate...")
	    getch()
	    quit()

	# Enable Dynamixel Torque
	for i in xrange(1):
		while 1:
			dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, i+1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
		
			if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
			    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
			elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
			    dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))
			else:
			    print("Dynamixel %d has been successfully connected" %(i+1))
			    break

	

def listener():

	rospy.init_node('listener3', anonymous=True)
	rospy.Subscriber('tester2', Data, callback)
	rospy.spin()

if __name__ == '__main__':
	dxl_init()
	torque_ctrl()
	listener()

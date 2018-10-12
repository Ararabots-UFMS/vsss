import bluetooth
from ctypes import *

# DIRECTION values are in range 4 to 7
# SPEED values both are in range 0 to 255

SET_PID_OP_BYTE = 12
ANGLE_CORRECTION_OP_BYTE = 8
CW_BYTE = 0
CCW_BYTE = 3
WALK_FORWARD_BYTE = 4
WALK_BACKWARDS_BYTE = 7
WALK_RIGHT_FRONT_LEFT_BACK_BYTE = 5
WALK_RIGHT_BACK_LEFT_FRONT_BYTE = 6

class Sender():

    def __init__ (self, robotId, bluetoothId, port=0x1001):
        # robot Id
        self.robotId = robotId
        # bluetooth mac
        self.bluetoothId = bluetoothId
        self.port = port
        self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

    def connect(self):
        """Connect to the robot"""
        try:
            self.sock.connect((self.bluetoothId, self.port))
        except:
            print "Connect error robot: ", self.robotId

    def send_movement_package(self, package_array, isHardwareCorretion = False):
        """
            Receives an array of angle and speed, if the correction is in hardware
            otherwise, just the wheels speed  
        """
        if isHardwareCorretion:
            send_angle_corretion(package_array[0], package_array[1])
        else:
            sendPacket(package_array[0], package_array[1])

    def sendPacket(self, leftWheel, rightWheel):
        """Recive the speed, get the first byte and then send the msg to the robot"""
        directionByte = self.getDirectionByte(leftWheel, rightWheel)
        left, right = self.normalizeWheels(leftWheel, rightWheel)
        try:
            self.sock.send(c_ubyte(directionByte))
            self.sock.send(c_ubyte(left))
            self.sock.send(c_ubyte(right))
        except:
            print "Packet error robot: ", self.robotId


    def send_set_pid_packet(self, KP, KI, KD):
        try:
            self.sock.send(c_ubyte(SET_PID_OP_BYTE))
            self.sock.send(c_ubyte(KP))
            self.sock.send(c_ubyte(KI))
            self.sock.send(c_ubyte(KD))
        except:
            print "Packet error robot: ", self.robotId

    def send_angle_corretion(self, theta, speed):
        try:
            correct_theta, orientation = self.get_angle_orientation_and_correction(theta)
            self.sock.send(c_ubyte(ANGLE_CORRECTION_OP_BYTE+orientation))
            self.sock.send(c_ubyte(correct_theta))
            self.sock.send(c_ubyte(speed))
        except:
            print "Packet error robot: ", self.robotId

    def get_angle_orientation_and_correction(self, theta):
        if theta < 180:
            return theta, CW_BYTE
        else:
            return 360-theta, CCW_BYTE

    def getDirectionByte(self, leftWheel, rightWheel):
        """Return the first byte that represents the robot direction"""
        if leftWheel >= 0 and rightWheel >= 0:
            return WALK_FORWARD_BYTE
        elif leftWheel < 0 and rightWheel < 0:
            return WALK_BACKWARDS_BYTE
        elif leftWheel < 0:
            return WALK_RIGHT_FRONT_LEFT_BACK_BYTE
        else:
            return WALK_RIGHT_BACK_LEFT_FRONT_BYTE

    def closeSocket(self):
        """Close the socket"""
        self.sendPacket(0, 0)
        self.sock.close()

    def normalizeWheels(self, leftWheel, rightWheel):
        """Normalize speed to 255"""
        return 255 if abs(leftWheel) > 255 else abs(leftWheel), 255 if abs(rightWheel) > 255 else abs(rightWheel)

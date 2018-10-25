from rospy import logfatal
import bluetooth
import math
import struct
from time import sleep
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
MAX_CONNECTION_ATTEMPT = 3

class Sender():

    def __init__ (self, robotId, bluetoothId, port=0x1001):
        # robot Id
        self.robotId = robotId
        # bluetooth mac
        self.bluetoothId = bluetoothId
        self.port = port
        self.sock = None

    def connect(self):
        """Connect to the robot"""

    	attempt = 1
        while (attempt <= MAX_CONNECTION_ATTEMPT):
            self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            try:
                self.sock.connect((self.bluetoothId, self.port))
            except IOError:
                logfatal("Unable to connect to "+self.bluetoothId+", waiting 5 seconds")
                self.sock.close()
                sleep(5)
            else:
                self.sock.setblocking(False)
                logfatal("Opened bluetooth device at "+str(self.port)+" after "+ str(attempt)+" attempts")
                break;
            attempt += 1

    def send_movement_package(self, package_array, isHardwareCorretion = False):
        """
            Receives an array of angle and speed, if the correction is in hardware
            otherwise, just the wheels speed
        """
        if isHardwareCorretion:
            self.send_angle_corretion(package_array[0], package_array[1])
        else:
            self.sendPacket(package_array[0], package_array[1])

    def sendPacket(self, leftWheel, rightWheel):
        """Recive the speed, get the first byte and then send the msg to the robot"""
        directionByte = self.getDirectionByte(leftWheel, rightWheel)
        left, right = self.normalizeWheels(leftWheel, rightWheel)
        try:
            self.sock.send(c_ubyte(directionByte))
            self.sock.send(c_ubyte(left))
            self.sock.send(c_ubyte(right))
        except Exception as e:
            print "Packet error robot: ", self.robotId, " E: ", e


    def send_float(self, n_float):
        for byte in struct.pack("!f", n_float):
            self.sock.send(c_ubyte(ord(byte)))

    def send_set_pid_packet(self, KP, KI, KD):
        try:
            self.sock.send(c_ubyte(SET_PID_OP_BYTE))
            self.send_float(KP)
            self.send_float(KI)
            self.send_float(KD)
        except Exception as e:
            print "Packet error robot: ", self.robotId, " E: ", e

    def send_angle_corretion(self, theta, speed, rad=True):
        try:
            correct_theta, orientation = self.get_angle_orientation_and_correction(theta, rad)
            self.sock.send(c_ubyte(ANGLE_CORRECTION_OP_BYTE+orientation))
            print "c", correct_theta, " ", speed
            self.sock.send(c_ubyte(correct_theta))
            self.sock.send(c_ubyte(speed))
        except Exception as e:
            print "Packet error robot: ", self.robotId, " E: ", e

    def get_angle_orientation_and_correction(self, angle, rad=True):
        tmp = angle
        if rad:
            tmp = 180.0*angle/math.pi
        if 0 <= tmp <= 180:
            return int(tmp), CW_BYTE
        else:
            return abs(int(tmp)), CCW_BYTE

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

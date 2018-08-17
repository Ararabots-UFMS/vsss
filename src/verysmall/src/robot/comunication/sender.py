import bluetooth
from ctypes import *

# DIRECTION values are in range 4 to 7
# SPEED values both are in range 0 to 255

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

    def getDirectionByte(self, leftWheel, rightWheel):
        """Return the first byte that represents the robot direction"""
        if leftWheel >= 0 and rightWheel >= 0:
            return 4
        elif leftWheel < 0 and rightWheel < 0:
            return 7
        elif leftWheel < 0:
            return 5
        else:
            return 6

    def closeSocket(self):
        """Close the socket"""
        self.sendPacket(0, 0)
        self.sock.close()

    def normalizeWheels(self, leftWheel, rightWheel):
        """Normalize speed to 255"""
        return 255 if abs(leftWheel) > 255 else abs(leftWheel), 255 if abs(rightWheel) > 255 else abs(rightWheel)
import bluetooth
from ctypes import *

DIRECTION = 0
SPEED_L = 1
SPEED_R = 2


class Sender():

    def __init__ (self, robotId, bluetoothId, port):
        self.robotId = robotId
        self.bluetoothId = bluetoothId
        self.port = port
        self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

    def connect(self):
        self.sock.connect((self.bluetoothId, self.port))

    def sandPacket(self, msg):
        values = map(int, msg.split())
        if len(values) == 3:
            # DIRECTION values are in range 4 to 7
            # SPEED values both are in range 0 to 255
            self.sock.send(c_ubyte(values[DIRECTION]))
            self.sock.send(c_ubyte(values[SPEED_L]))
            self.sock.send(c_ubyte(values[SPEED_R]))
        else:
            self.invalidPacket()

    def closeSocket(self):
        self.sock.close()

    def invalidPacket(self):
        # Stops the robot
        self.sandPacket("4 0 0")
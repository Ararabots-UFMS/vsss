import bluetooth

class Client():

    def __init__ (self, robotId, bluetoothId, port):
        self.robotId = robotId
        self.bluetoothId = bluetoothId
        self.port = port
        self.sock = bluetooth.BluetoothSocket(bluetooth.L2CAP)

    def connect(self):
        sock.connect((self.bluetoothId, self.port))

    def sandPacket(self, direction, speedLeft, speedRight):
        packet = bytearray()
        pecket.append(direction)
        pecket.append(speedLeft)
        pecket.append(speedRight)
        sock.send(packet)

    def closeSocket(self):
        sock.close()

    def reconnect(self):
        pass
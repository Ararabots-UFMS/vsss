import bluetooth

class Client():

    def __init__ (self, robotId, bluetoothId, port):
        self.robotId = robotId
        self.bluetoothId = bluetoothId
        self.port = port
        self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

    def connect(self):
        self.sock.connect((self.bluetoothId, self.port))

    def sandPacket(self, direction, speedLeft, speedRight):
        packet = bytearray()
        packet.append(direction)
        packet.append(speedLeft)
        packet.append(speedRight)
        self.sock.send(packet)

    def closeSocket(self):
        self.sock.close()

    def reconnect(self):
        pass
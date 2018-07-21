from sender import Sender
# initialize the client
lucio = Sender(robotId=1, bluetoothId="20:15:04:09:77:68")
robotop = Sender(robotId=2, bluetoothId="20:15:04:09:70:80")
# connect the client
lucio.connect()
robotop.connect()

for i in xrange(10):
    lucio.sendPacket(100, -100)
    robotop.sendPacket(-100, 100)

lucio.closeSocket()
robotop.closeSocket()

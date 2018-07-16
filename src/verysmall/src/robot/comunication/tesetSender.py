from sender import Sender
# initialize the client
client = Sender(robotId=1, bluetoothId="20:15:04:09:77:68", port=0x1001)
# connect the client
client.connect()

for i in xrange(10):
    client.sandPacket("4 100 100")

client.sandPacket("4 100")

for i in xrange(100):
    client.sandPacket("4 100 100")
    
# stop the robot and close the socket
client.sandPacket("4 0 0")
client.closeSocket()

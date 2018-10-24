from sender import Sender
from time import sleep
# initialize the client
dumbo = Sender(robotId=1, bluetoothId="20:15:04:09:70:95")
#robotop = Sender(robotId=2, bluetoothId="20:15:04:09:70:80")
# connect the client
dumbo.connect()
#robotop.connect()

#for i in xrange(10):
#dumbo.send_set_pid_packet(2.0, 0.01, 0.5)
    #robotop.sendPacket(-100, 100)
dumbo.sendPacket(255,255)
sleep(1)
dumbo.closeSocket()
#robotop.closeSocket()

from sender import Sender
from time import sleep
# initialize the client
dumbo = Sender(robotId=1, bluetoothId="84:0D:8E:0C:92:A2")
#robotop = Sender(robotId=2, bluetoothId="20:15:04:09:70:80")
# connect the client
dumbo.connect()
#robotop.connect()

#for i in xrange(10):
dumbo.send_set_pid_packet(2.0, 0.01, 0.5)
dumbo.send_angle_corretion(0.0, 0, True)

#robotop.sendPacket(-100, 100)
sleep(60)
dumbo.closeSocket()
#robotop.closeSocket()

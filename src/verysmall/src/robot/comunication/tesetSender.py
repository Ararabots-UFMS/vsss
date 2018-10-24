from sender import Sender
from time import sleep

# initialize the client
dumbo = Sender(robotId=1, bluetoothId="20:15:04:09:70:95")
robotop = Sender(robotId=2, bluetoothId="84:0D:8E:0C:92:A2")
#robotop = Sender(robotId=2, bluetoothId="84:0D:8E:0C:92:8E")
# connect the client
#robotop.sock.shutdown(0)
dumbo.connect()
robotop.connect()
sleep(1)

robotop.sendPacket(0,0)

for i in xrange(30):
	dumbo.sendPacket(255, -255)
	robotop.sendPacket(255, -255)
	sleep(0.05)

# dumbo.send_set_pid_packet(2.0, 0.01, 0.5)
# dumbo.send_angle_corretion(0.0, 0, True)
#robotop.sendPacket(0,0)
dumbo.closeSocket()
robotop.closeSocket()

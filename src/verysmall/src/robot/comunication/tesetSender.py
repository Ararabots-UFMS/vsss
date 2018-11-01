from sender import Sender
from time import sleep

# initialize the client
dumbo = Sender(robotId=1, bluetoothId="84:0D:8E:0C:92:A2")
#robotop = Sender(robotId=2, bluetoothId="20:15:04:09:70:80")
# connect the client
#robotop.sock.shutdown(0)
dumbo.connect()
# robotop.connect()
# sleep(1)

# robotop.sendPacket(0,0)

for i in xrange(100):
# dumbo.send_set_pid_packet(2.0, 0.01, 0.5)
    # robotop.sendPacket(-100, 100)
	dumbo.sendPacket(250,250)
	sleep(0.05)
sleep(5)
dumbo.closeSocket()
# robotop.closeSocket()

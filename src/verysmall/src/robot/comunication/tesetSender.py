from sender import Sender
# initialize the client
dumbo = Sender(robotId=1, bluetoothId="B4:E6:2D:B5:B9:6B")
#robotop = Sender(robotId=2, bluetoothId="20:15:04:09:70:80")
# connect the client
dumbo.connect()
#robotop.connect()

#for i in xrange(10):
dumbo.send_set_pid_packet(2.0, 0.01, 0.5)
    #robotop.sendPacket(-100, 100)

dumbo.closeSocket()
#robotop.closeSocket()

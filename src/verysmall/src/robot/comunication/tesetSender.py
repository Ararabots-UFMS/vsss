from sender import Sender

client = Sender(robotId=1, bluetoothId="20:15:04:09:70:80", port=0x1001)
client.connect()
while 1:
    client.sandPacket(1, 100, 100)
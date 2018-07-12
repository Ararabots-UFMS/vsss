from client import Client

cliente = Client(robotId=1, bluetoothId="20:15:04:09:70:80", port=0x1001)
cliente.connect()
while 1:
    cliente.sandPacket(1, 100, 100)
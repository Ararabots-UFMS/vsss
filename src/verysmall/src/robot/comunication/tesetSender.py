from sender import Sender
from time import sleep
import cv2

# initialize the client
# robot = Sender(robotId=1, bluetoothId="84:0D:8E:0C:93:16") # umbabarauma
# robot = Sender(robotId=1, bluetoothId="84:0D:8E:0C:92:A2") # araraquara
robot = Sender(robotId=1, bluetoothId="84:0D:8E:0D:BA:32") # jeremias
robot.connect()

speed0 = 70
cv2.namedWindow("controle")

while True:

    key = 0xFF & cv2.waitKey(1)

    if key == ord('s'):
        robot.sendPacket(0, 0)
    elif key == ord('b'):
        robot.sendPacket(0, 0)
        sleep(0.2)
        robot.send_angle_corretion(0, -150)
    elif key == ord('p'):
            robot.send_angle_corretion(0, 150)
    elif key == ord('r'):
        print "Rotating"
        robot.sendPacket(-25, 25)
    elif key == ord('c'):
        kp, ki, kd = raw_input("Please type kP kI kD: ").split(' ')
        kp = float(kp);
        ki = float(ki);
        kd = float(kd);
        robot.send_set_pid_packet(kp, ki, kd)
        print "sent!"
    elif key == ord('q'):
        cv2.destroyWindow("controle")
        break

robot.closeSocket()
#robotop.closeSocket()

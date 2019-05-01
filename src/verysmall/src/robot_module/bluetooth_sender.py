import rospy
import sys
from verysmall.msg import motor_speed
from verysmall.srv import manage_mac


class Bluetooth():
    """docstring for Bluetooth"""

    def __init__(self):
        self.robots_motor_speed = {}

        self.service = rospy.Service('manage_mac', manage_mac, self.manage_mac_address)

        rospy.init_node('bluetooth')
        rospy.spin()

    def manage_mac_address(self, req):
        success = True
        if req.operation == 1:  # Register the HW Address
            try:
                rospy.Subscriber('robot_'+str(req.robot_id), motor_speed, self.send_motor_package)
            except rospy.ROSException as exc:
                print("Service did not process request: " + str(exc))

        else:
            rospy.logfatal((req))
        return success

    def send_motor_package(self, data):
        rospy.loginfo("Sending to " + data[0] + "power: ")


if __name__ == "__main__":
    Bluetooth()

#!/usr/bin/env python
# license removed for brevity
import rospy
from robotiq_vacuum_grippers_control.msg import RobotiqVacuumGrippers_robot_output

def talker():
    pub = rospy.Publisher('/RobotiqVacuumGrippersRobotOutput', RobotiqVacuumGrippers_robot_output, queue_size=10)
    rospy.init_node('test', anonymous=False)
    rate = rospy.Rate(10) # 10hz

    msg = RobotiqVacuumGrippers_robot_output
    pub.publish(msg)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

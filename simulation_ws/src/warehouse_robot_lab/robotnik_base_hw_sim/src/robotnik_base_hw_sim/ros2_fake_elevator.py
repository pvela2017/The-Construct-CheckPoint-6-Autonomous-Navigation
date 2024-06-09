#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from robotnik_msgs.msg import SetElevatorActionGoal

def callback(data):
    rospy.loginfo("elevator up!")
    pub = rospy.Publisher("/robot/robotnik_base_control/set_elevator/goal", SetElevatorActionGoal, queue_size=10)
    up = SetElevatorActionGoal()
    up.goal.action.action = 1
    pub.publish(up)

def callback2(data):
    rospy.loginfo("elevator down!")
    pub2 = rospy.Publisher("/robot/robotnik_base_control/set_elevator/goal", SetElevatorActionGoal, queue_size=10)
    down = SetElevatorActionGoal()
    down.goal.action.action = -1
    pub2.publish(down)
    
def listener():

    rospy.init_node('ros2_elevator_listener', anonymous=True)

    pub = rospy.Publisher("/robot/robotnik_base_control/set_elevator/goal", SetElevatorActionGoal, queue_size=10)
    sub1 = rospy.Subscriber("elevator_up", Empty, callback)
    sub2 = rospy.Subscriber("elevator_down", Empty, callback2)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
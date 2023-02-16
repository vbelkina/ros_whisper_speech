"""
basic Exit class which will close the open nodes that are subscribed to the exit message
"""

import rospy
from std_msgs.msg import String

class Exit():

    def __init__(self):
        self.speak_pub = rospy.Publisher("/speak", String, queue_size=10)

    def exit_program(self, node_name):
        """
        exit the program safely and get the robot to say bye bye...  
        """
        rospy.loginfo(f"{node_name} shutting down...")
        rospy.signal_shutdown("Stop")
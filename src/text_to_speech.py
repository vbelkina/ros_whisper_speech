#!/usr/bin/env python3

"""
text_to_speech - get published messages and use pyttsx3 to convert to speech. 
It uses espeak. 
"""

import rospy
from std_msgs.msg import String
import pyttsx3

class text_to_speech():

    def __init__(self):
        self.engine = pyttsx3.init()
        self.engine.setProperty("rate", 178)
        
        self.text_sub = rospy.Subscriber("/speak", String, self.speak_cb) 
        
    def speak_cb(self, msg):
        """
        get published message and get it to be spoken
        """
        print(msg)
        text = msg.data
        self.engine.say(text)
        self.engine.runAndWait()


if __name__== '__main__':
    rospy.init_node("text_to_speech")
    text_to_speech()
    rospy.spin()
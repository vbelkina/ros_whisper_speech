#!/usr/bin/env python3

"""
text_to_speech - get published messages and use pyttsx3 to convert to speech. 
It uses espeak so doesn't sound great...
might switch to google text to speech and just save each time as an mp3...
"""

import rospy
from std_msgs.msg import String, Bool
from gtts import gTTS
import os
from exit import Exit


class text_to_speech():

    def __init__(self):
        self.language = 'en'
        self.tld = 'com.au'
        self.text_sub = rospy.Subscriber("/speak", String, self.speak_cb) 
        self.exit_sub = rospy.Subscriber("/exit", Bool, self.exit_cb)
        self.is_speaking = rospy.Publisher("/speaking", Bool, queue_size=1)
        self.node_name = "[TTS]"
        
    def speak_cb(self, msg):
        """
        get published message and get it to be spoken
        """
        rospy.loginfo(f"{self.node_name} {msg}")
        text = msg.data

        # eventually check if there is internet connection
        self.google(text)

    def google(self, text):
        speech = gTTS(text= text, lang=self.language, tld=self.tld, slow=False)
         # Saving the converted audio in a mp3 file named
        speech.save("speak.mp3")
        
        self.is_speaking.publish(True)

        # Playing the converted file
        os.system("mpg123 -q speak.mp3")

        self.is_speaking.publish(False)
    
    def exit_cb(self, msg):
        if msg.data:
            speech = gTTS(text= "bye bye", lang=self.language, tld=self.tld, slow=False)
            # Saving the converted audio in a mp3 file named
            speech.save("speak.mp3")
            
            # Playing the converted file
            os.system("mpg123 -q speak.mp3")
            Exit().exit_program(self.node_name)


if __name__== '__main__':
    rospy.init_node("text_to_speech")
    text_to_speech()
    rospy.spin()
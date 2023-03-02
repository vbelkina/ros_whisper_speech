#!/usr/bin/env python3

"""
initialize a speech recognizer that will listen for a wake word "hello robot" 
and when it hears it, it will start to listen for other commands such as to drive/go forward, 
turn left/right, stop, exit. It uses python's speech recognition library to connect
to the microphone and listen and also to connect to openai's whisper speech to text api. 
"""

import rospy
import speech_recognition as sr  
import re
from std_msgs.msg import String, Bool 
from exit import Exit
import signal

# rospy.init_node('whisper_sr')

class WSR(): 
    
    def __init__(self, mic):
      """ 
      mic: instance of sr.Recognizer() from wake node -- should hopefully have the set values from the wake node??
      """
      # set up recognizer and thresholds
      self.r = mic
      self.rate = rospy.Rate(10)
      self.node_name = "[WHISPER]"
      self.is_speaking = False

      self.command_pub = rospy.Publisher("/command", String, queue_size=10)
      self.speak_pub = rospy.Publisher("/speak", String, queue_size=1)
      self.exit_sub = rospy.Subscriber("/exit", Bool, self.exit_cb)
      self.is_speaking_sub = rospy.Subscriber("/speaking", Bool, self.speaking_cb)

    def speaking_cb(self,msg):
       self.is_speaking = msg.data
       
    def listen_for_command(self, time_since):
      """
      after getting "hello robot", listen for the next commands for 60 sec after which it will
      return to listening for the wake word.
      """
      while rospy.Time.now().to_sec() - time_since < 60: 
        self.speak_pub.publish("listening for a command...")
        if not self.is_speaking: 
          rospy.loginfo(f"{self.node_name} time passed: {(rospy.Time.now().to_sec() - time_since)}")
          with sr.Microphone() as source:  
                audio = self.r.listen(source, phrase_time_limit=5, timeout=5)

          try:  
            result = self.r.recognize_whisper(audio)
            # remove punctuation and lowercase result
            result = re.sub(r'[^\w\s]', '', result.lower())
            if result: 
              rospy.loginfo(f"{self.node_name} publishing command {result}")
              self.command_pub.publish(result)
          except sr.UnknownValueError:  
            rospy.logwarn(f"{self.node_name} whisper could not understand audio")  
          except sr.RequestError as e:  
            rospy.logwarn(f"{self.node_name} whisper error; {0}".format(e))  

          self.rate.sleep()
        else: 
            rospy.loginfo(f"{self.node_name} waiting for the robot to finish speaking")

    def exit_cb(self, msg):
      if msg.data:
          Exit().exit_program(self.node_name)

      
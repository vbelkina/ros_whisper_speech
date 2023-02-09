#!/usr/bin/env python3

import rospy
import speech_recognition as sr  
from gtts import gTTS
import re
from std_msgs.msg import String

rospy.init_node('whisper_sr')

class speech_recognizer(): 
  def __init__(self):
    # set up recognizer and thresholds
    self.r = sr.Recognizer()
    self.r.dynamic_energy_threshold = False
    self.r.energy_threshold = 400
    self.time_since_hello_robot = rospy.Time.now().to_sec()

    self.command_pub = rospy.Publisher("/command", String, queue_size=10)

    self.calibrate_mic()

  def calibrate_mic(self):
    with sr.Microphone() as source:  
        rospy.loginfo("Please wait. Calibrating microphone...")  
        # listen for 5 seconds and create the ambient noise energy level  
        self.r.adjust_for_ambient_noise(source, duration=5)  

  def listen_for_command(self):
    while rospy.Time.now().to_sec() - self.time_since_hello_robot < 60: 
      print("time passed:", rospy.Time.now().to_sec() - self.time_since_hello_robot, " passed: ", rospy.Time.now().to_sec() - self.time_since_hello_robot < 60)
      with sr.Microphone() as source:  
          rospy.loginfo("listening for a command...")
          audio = self.r.listen(source)

      try:  
        result = self.r.recognize_whisper(audio)
        result = re.sub(r'[^\w\s]', '', result.lower())
        rospy.loginfo("Publishing command {}".format(result))
        self.command_pub.publish(result)
      except sr.UnknownValueError:  
        rospy.logwarn("Whisper could not understand audio")  
      except sr.RequestError as e:  
        rospy.logwarn("Whisper error; {0}".format(e))  
      rate.sleep()
  
  def listen_for_hello_robot(self):
    rospy.loginfo("listening for keyword: hello robot")
    with sr.Microphone() as source: 
      audio = self.r.listen(source)
    
    try: 
      result = self.r.recognize_whisper(audio)
      result = re.sub(r'[^\w\s]', '', result.lower().strip())
      rospy.loginfo("whisper heard: {}".format(result))

      accepted_hello_robots = ["hello robot", "hello robert", "alo robot", "alo robert"]
      
      if result in accepted_hello_robots: 
        # print("entered here")
        self.time_since_hello_robot = rospy.Time.now().to_sec()
        self.listen_for_command()
      else:
        rospy.loginfo("still listening for keyword: hello robot...")
    except sr.UnknownValueError: 
      rospy.loginfo("could not understand...")
    except sr.RequestError as e:  
      rospy.logwarn("whisper error; {0}".format(e))  

  def exit_program(self):
      rospy.loginfo("whisper_sr.py safely shutting down...")
      rospy.signal_shutdown("Stop")

srr = speech_recognizer()
rate = rospy.Rate(10)

while not rospy.is_shutdown(): 
    srr.listen_for_hello_robot() 
    rospy.on_shutdown(srr.exit_program)
    rate.sleep()



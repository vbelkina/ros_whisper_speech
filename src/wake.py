#!/usr/bin/env python3


import rospy
import speech_recognition as sr  
import re
from std_msgs.msg import String, Bool
from whisper_sr import WSR
from exit import Exit

class Wake():

    def __init__(self):
        # set up recognizer and thresholds
        self.r = sr.Recognizer()
        self.r.dynamic_energy_threshold = False
        self.r.energy_threshold = 400
        self.time_since_hello_robot = rospy.Time.now().to_sec()
        self.device_index = 0
        self.node_name = "[WAKE]"

        self.command_pub = rospy.Publisher("/command", String, queue_size=10)
        self.speak_pub = rospy.Publisher("/speak", String, queue_size=1)
        self.exit_sub = rospy.Subscriber("/exit", Bool, self.exit_cb)

        self.calibrate_mic()

    def calibrate_mic(self):
        """
        calibrate mic by listening for ambient noise for 5 sec
        """
        with sr.Microphone() as source:  
            rospy.loginfo("[WAKE] Please wait. Calibrating microphone...")  
            # listen for 5 seconds and create the ambient noise energy level  
            self.r.adjust_for_ambient_noise(source, duration=5)  

    def wake_up(self):
        """
        listen for "hello robot" before letting the robot take commands
        """
        rospy.loginfo("{self.node_name} listening for keyword: hello robot")
        with sr.Microphone() as source: 
            audio = self.r.listen(source)   
        
        try: 
            result = self.r.recognize_whisper(audio)
            result = re.sub(r'[^\w\s]', '', result.lower().strip())
            rospy.loginfo("{self.node_name} whisper heard: {}".format(result))

            # list of possible wake phrases that sound similar to hello robot
            accepted_hello_robots = ["hello robot", "hello robert", "alo robot", "alo robert", "hello rover"]
            
            if result in accepted_hello_robots: 
                # publish text that robot will say
                self.speak_pub.publish("hi")
                # record the time when the wake phrases was heard
                self.time_since_hello_robot = rospy.Time.now().to_sec()
                # will create a new instance of speech_recognizer that will listen for commands for 60 sec
                WSR(self.r).listen_for_command(rospy.Time.now().to_sec())
            else:
                # if no wake word was heard, then restart the loop
                self.speak_pub.publish("still listening...")
                rospy.loginfo("{self.node_name} still listening for keyword: hello robot...")
        except sr.UnknownValueError: 
            self.speak_pub.publish("{self.node_name} sorry, I couldn't understand what you said...")
            rospy.loginfo("{self.node_name} could not understand...")
        except sr.RequestError as e:  
            rospy.logwarn("{self.node_name} whisper error; {0}".format(e))  

    def exit_cb(self, msg):
        if msg.data:
            Exit().exit_program(self.node_name)
            

rate = rospy.Rate(10)
waker = Wake()

while not rospy.is_shutdown():
    waker.wake_up()
    rate.sleep()

#!/usr/bin/env python3

"""
take in spoken command from whisper_sr and check if it is in the commands.json file
if it is, then it will execute the command. And if not, it will say it did not recognize 
the command. 
"""

import rospy 
import json 
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from exit import Exit

rospy.init_node('commands')

class Commands():

    def __init__(self):
        self.command_sub = rospy.Subscriber("/command", String, self.command_cb) 
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.speak_pub = rospy.Publisher("/speak", String, queue_size=1)
        self.exit_pub = rospy.Publisher("/exit", Bool, queue_size=1)
        self.exit_sub = rospy.Subscriber("/exit", Bool, self.exit_cb)

        self.node_name = "[COMMANDS]"
        self.executing = f"{self.node_name} Executing: "
        self.not_recognized = f"{self.node_name} sorry, I didn't recognize the command "

        self.commands = []

        with open('/home/nika/catkin_ws/src/whisper/commands.json') as file:
            self.parsed_json = json.load(file)

    def command(self):
        """
        executes the command received from the whisper_sr.py node. 
        if the command is correct and found within the json file, a cmd_vel message 
        will be published, otherwise, it will say that the command was not recognized. 
        """
        twist = Twist()
        current_command = self.commands.pop() 
        command_list = current_command.strip().split()

        if command_list[0] == "stop":
            self.speak_pub.publish("stopping")
            rospy.loginfo(f"{self.executing}{current_command}")
            self.cmd_pub.publish(twist)
        elif command_list[0] == "exit":
            self.exit_pub.publish(True)
        elif command_list[0] in self.parsed_json: 
            if command_list[1] in self.parsed_json[command_list[0]]:
                twist.linear.x = self.parsed_json[command_list[0]][command_list[1]]["linear.x"]
                twist.linear.y = self.parsed_json[command_list[0]][command_list[1]]["linear.y"]
                twist.linear.z = self.parsed_json[command_list[0]][command_list[1]]["linear.z"]
                twist.angular.x = self.parsed_json[command_list[0]][command_list[1]]["angular.x"]
                twist.angular.y = self.parsed_json[command_list[0]][command_list[1]]["angular.y"]
                twist.angular.z = self.parsed_json[command_list[0]][command_list[1]]["angular.z"]
                
                rospy.loginfo(f"{self.executing} {current_command}")
                self.speak_pub.publish("ok, will do.")
                self.cmd_pub.publish(twist)
            else: 
                self.speak_pub.publish("{self.not_recognized} {current_command}")
                rospy.logwarn(f"{self.not_recognized} {current_command}")
        else: 
            self.speak_pub.publish("{self.not_recognized} {current_command}")
            rospy.logwarn(f"{self.not_recognized} {current_command}")

    def command_cb(self, msg):
        """
        received String messages that will give commands to the robot
        """
        
        rospy.loginfo(f"{self.node_name} Command {msg.data} received")
        if msg.data != "": 
            self.commands.append(msg.data)

        if self.commands: 
            self.command()
    
    def exit_cb(self, msg):
        if msg.data:
            twist = Twist()
            self.cmd_pub.publish(twist)
            Exit().exit_program(self.node_name)


if __name__ == '__main__':
    command = Commands()
    # rospy.on_shutdown(command.exit_program)
    rospy.spin()


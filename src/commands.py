#!/usr/bin/env python3

import rospy 
import json 
from std_msgs.msg import String
from geometry_msgs.msg import Twist

rospy.init_node('commands')

class Commands():

    def __init__(self):
        self.command_sub = rospy.Subscriber("/command", String, self.command_cb) 
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.speak_pub = rospy.Publisher("/speak", String, queue_size=1)

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
            rospy.loginfo("Executing {}".format(current_command))
            self.cmd_pub.publish(twist)
            
        elif command_list[0] in self.parsed_json: 
            if command_list[1] in self.parsed_json[command_list[0]]:
                twist.linear.x = self.parsed_json[command_list[0]][command_list[1]]["linear.x"]
                twist.linear.y = self.parsed_json[command_list[0]][command_list[1]]["linear.y"]
                twist.linear.z = self.parsed_json[command_list[0]][command_list[1]]["linear.z"]
                twist.angular.x = self.parsed_json[command_list[0]][command_list[1]]["angular.x"]
                twist.angular.y = self.parsed_json[command_list[0]][command_list[1]]["angular.y"]
                twist.angular.z = self.parsed_json[command_list[0]][command_list[1]]["angular.z"]
                
                rospy.loginfo("Executing {}".format(current_command))
                self.speak_pub.publish("ok, will do.")
                self.cmd_pub.publish(twist)
            else: 
                rospy.logwarn("{} is not recognized".format(current_command))
        else: 
            rospy.logwarn("{} not recognized".format(current_command))

    def command_cb(self, msg):
        """
        received String messages that will give commands to the robot
        """
        
        rospy.loginfo("Command {} received".format(msg.data))
        if msg.data != "": 
            self.commands.append(msg.data)
        if "exit" in msg.data: 
            self.exit_program()

        if self.commands: 
            self.command()
    
    def exit_program(self):
        """
        exit the program safely -- stop the robot and tell the node to shutdown after receiving 
        the ctrl+c command. 
        """
        rospy.loginfo("commands.py safely shutting down...")
        self.cmd_pub.publish(Twist())
        rospy.signal_shutdown("Stop")


if __name__ == '__main__':
    command = Commands()
    rospy.on_shutdown(command.exit_program)
    rospy.spin()


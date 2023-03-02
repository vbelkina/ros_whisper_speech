from whisper_sr import WSR
from text_to_speech import text_to_speech
from exit import Exit
from wake_up import WakeUp
from commands import Commands
import rospy

while not rospy.is_shutdown():
    WakeUp().wake_up()
    

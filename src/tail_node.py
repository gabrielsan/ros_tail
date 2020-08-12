#!/usr/bin/env python
import rospy
import roslib
import sys
import time
import serial
from math import pi
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import Float64
from std_msgs.msg import String

# configure the serial connection
try:
    ser = serial.Serial('/dev/ttyUSB0', 115200)

    ser.isOpen()
    print 'Connected to rear bridge'

except IOError: # if port is already opened, close it and open it again and print message
    print ("port was already open, closing....")  
    ser.close()
    ser.open() 
    print 'Connected to rear bridge'

time.sleep(1)


class tail_pluto:
    

    # Intializes everything
    def __init__(self, script_path):
  

    	#  rospy.init_node('tail_listener')
    	rospy.init_node('tail', anonymous=True)


    	# subscribe to sound player
    	rospy.Subscriber("robotsound", String, self.callback, queue_size=1)
    	

	# code to run when shutdown
        rospy.on_shutdown(self.cleanup)

        
    	# starts the node
    	rospy.spin()





    def cleanup(self):
        
        rospy.loginfo("Shutting down tail node...")

    def callback(self, data):
	rospy.loginfo(data)
        rospy.loginfo("received sound...wagging tail")
        ser.write('h\r\n')












if __name__ == '__main__':

    try:
        tail_pluto(sys.path[0])
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("tail node terminated.")


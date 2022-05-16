#!/usr/bin/env python

import rospy
import rospkg # Used to find path to data file
import numpy as np # Used for simplified data file reading
import geometry_msgs.msg as gmsg # Used with publishing transforms
from radiation_msgs.msg import DoseRate # Used for communicating radiation information
import tf2_ros as tf2 # Used to publish transforms



class filePublisher(object):
    """Publishes a text file of data to demonstrate use of the radiation costmap layer"""
    def __init__(self):
        rospy.init_node("radiationFromFile") # Init node called "radiationFromFile"
        radiationTopic = rospy.get_param("~radiationName","radiationTopic") # Name of the topic containing radiation data

	self.radiationPublisher = rospy.Publisher(radiationTopic, DoseRate, queue_size=2) # Publisher of image messages carrying radiation information
        self.globalFrame = rospy.get_param("~globalFrame","map") # Set the frame in which the pointcloud should be referenced to
        self.childFrame = rospy.get_param("~childFrame","base_link") # Set the frame in which the pointcloud should be referenced to

	# Load data file
	# Data file format: x, y, z, counts
	package_path = rospkg.RosPack().get_path('radiation_layer') 
	data_file = '/data/radiation_sample.dat'
	self.radData = np.loadtxt(package_path + data_file, delimiter=',') # Data file is comma seperated
        self.nSamples = self.radData.shape[0]

        self.seq = 0

        rate = rospy.Rate(5) #Rate in Hz

        rospy.loginfo("Publishing radiation data")

        while not rospy.is_shutdown():
            if self.seq < self.nSamples:
                self.publishdata(self.radData[self.seq,:])
                rate.sleep()
            elif self.seq >= self.nSamples:
                self.publishdata(self.radData[self.nSamples-1,:])
                rate.sleep()
            else:
                rate.sleep()

    def publishdata(self, data):
        time = rospy.Time.now()
	# Update TF to match sensor location in x, y, z
        self.setTF(data[0], data[1], data[2], time)
	
	# Create blank image message
        message = DoseRate()
	# Populate header information
	message.header.seq = self.seq
	message.header.stamp = time
        message.header.frame_id = self.childFrame
        # Populate radiation information
        message.radiation_type = 4 # Gamma
        message.units = 0 # Counts per second
        message.integration_time = 1.0 # Second
	
        # Set rate value to match radiation measured
	message.rate = data[3]

        # Publish message
        self.radiationPublisher.publish(message)
        self.seq += 1

        if self.seq == self.nSamples:
                rospy.loginfo("Finished playing back data file")
	

    def setTF(self, x, y, z, time):
    	br = tf2.TransformBroadcaster()
        t = gmsg.TransformStamped()

        t.header.stamp = time
        t.header.frame_id = self.globalFrame
        t.child_frame_id = self.childFrame

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
	
	# Assume no rotation
    	t.transform.rotation.x = 0.0
    	t.transform.rotation.y = 0.0
    	t.transform.rotation.z = 0.0
    	t.transform.rotation.w = 1.0

        br.sendTransform(t)


if __name__ == "__main__":

    try:
        rospy.loginfo("Starting radiation data player")
        s = filePublisher()
        rospy.spin()
    except rospy.ROSInterruptException: pass

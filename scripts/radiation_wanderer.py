#!/usr/bin/env python

import rospy
import geometry_msgs.msg as gmsg # Used with publishing transforms
from radiation_msgs.msg import DoseRate # Used for communicating radiation information
import tf2_ros as tf2 # Used to publish transforms
import tf.transformations as tft
import math as maths



class filePublisher(object):
    """Publishes radiation intensity as a function of the distance from the origin,
    whilst a TF frame is gradually moved around the space"""
    def __init__(self):
        rospy.init_node("radiationFromFile") # Init node called "radiationFromFile"
        radiationTopic = rospy.get_param("~radiationName","radiationTopic") # Name of the topic containing radiation data

        self.radiationPublisher = rospy.Publisher(radiationTopic, DoseRate, queue_size=2) # Publisher of image messages carrying radiation information
        self.globalFrame = rospy.get_param("~globalFrame","map") # Set the frame in which the pointcloud should be referenced to
        self.childFrame = rospy.get_param("~childFrame","base_link") # Set the frame in which the pointcloud should be referenced to

        self.spiral_b = 0.1 # m

        self.nTurns = 10
        self.completion_time = 10 # seconds

        self.pub_rate = 5.0 # Hz

        self.z = 0 # m

        self.variance = 2
        self.radiation_source_intensity = 1000

        self.start_time = rospy.Time.now()

        rate = rospy.Rate(self.pub_rate) #Rate in Hz
        

        rospy.loginfo("Publishing radiation data")

        while not rospy.is_shutdown():
            time_now = rospy.Time.now()
            time_delta = time_now - self.start_time
            duration = time_delta.to_sec()

            angle, radius = self.calcSpiral(duration)
            
            self.publishRad(angle, radius, time_now)
            
            rate.sleep()
    

    def publishRad(self, angle, radius, time):
        radiation_intensity = self.radiationIntensity(radius)
        # Update TF to match sensor location in x, y, z
        self.setTF(angle, radius, time)
        
        # Create blank image message
        message = DoseRate()
        
        # Populate header information
        message.header.stamp = time
        message.header.frame_id = self.childFrame
        # Populate radiation information
        message.radiation_type = 4 # Gamma
        message.units = 0 # Counts per second
        message.integration_time = 1 / self.pub_rate # Second
        
        # Set rate value to match radiation measured
        message.rate = radiation_intensity
        
        # Publish message
        self.radiationPublisher.publish(message)
    

    def setTF(self, angle, radius, time):
        br = tf2.TransformBroadcaster()
        t = gmsg.TransformStamped()

        t.header.stamp = time
        t.header.frame_id = self.globalFrame
        t.child_frame_id = self.childFrame

        x, y = self.polar2cartesian(angle, radius)
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = self.z
    
        # Maintain orientation normal to origin
        quat = tft.quaternion_from_euler(0, 0, angle + maths.pi/2) # 
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        br.sendTransform(t)

    def calcSpiral(self, time):
        # Simple spiral pattern, r = b*theta, where b is a constant defining the spiral spacing

        # The TF frame should move at a set linear speed, therefore the rate of angle change should decrease over time
        angle = maths.sqrt(time / self.completion_time) * (2*maths.pi*self.nTurns) / self.completion_time
        radius = self.spiral_b * angle
        return angle, radius


    def polar2cartesian(self, theta, r):
        x = r * maths.cos(theta)
        y = r * maths.sin(theta)
        return x,y

    def radiationIntensity(self, r):
        # Gaussian with zero mean
        amplitude = self.radiation_source_intensity * maths.exp(-0.5 * (r / self.variance)**2)
        return amplitude



if __name__ == "__main__":

    try:
        rospy.loginfo("Starting radiation data player")
        s = filePublisher()
        rospy.spin()
    except rospy.ROSInterruptException: pass

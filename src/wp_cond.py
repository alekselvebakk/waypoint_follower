#!/usr/bin/env python3.7
import numpy as np
import rospy
import std_msgs


finish_flag = false 
wp = np.array()

class ConditionWP:
    def __init__(self):
        self.wp_list = np.array([])
        self.


        self.cond_wp_publish = rospy.Publisher('conditioned_wp', )

        rospy





        #Subscribing to the three filtered ultrasonic sensordata
        self.ultra_left_sub = rospy.Subscriber("ultrasound/left/filtered", Range, self.store_new_certain_measurement, callback_args="left")
        self.ultra_center_sub = rospy.Subscriber("ultrasound/center/filtered", Range, self.store_new_certain_measurement, callback_args="center")
        self.ultra_right_sub = rospy.Subscriber("ultrasound/right/filtered", Range, self.store_new_certain_measurement, callback_args="right")
        
        #Initializations
        self.last_certain_measurement = {"left": Range(), "center": Range(), "right": Range()}
        self.velref = PointStamped()
        self.velref.header.stamp = rospy.Time.now()
        self.velref.header.frame_id = "/ultrasound/center"
        self.velref_measurement_counter = 0 
#!/usr/bin/env python
import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
import std_msgs
from guidance.msg import Float
from geometry_msgs.msg import Point
import time

#This script is for conditioning the setpoint, current position and previous
#setpoint, to a numpy array and pass it into the guidance-system
#
class ConditionWP:
    def __init__(self):
        rospy.init_node('wps_conditioner')
        self.wp_list = [[0,0,1]]
        self.R_treshold = 2
        self.current_i = 0


        #Publish to topic cond_wps
        self.cond_wp_publish = rospy.Publisher('cond_wps', numpy_msg(Float), queue_size = 10)

        #Subscriber to waypoints
        self.wp_subscriber = rospy.Subscriber('wp_list', Point, callback = self.save_gotten_wps, queue_size = 10)
        #Subscriber for current position
        self.pos_subscriber = rospy.Subscriber('pos_coord', Point, self.save_gotten_pos)

    
    #Helping functions
    def error(wp_x, posx, wp_y, posy):
        errorx = wp_x-posx
        errory = wp_y-posy
        distance_from_wp = np.sqrt(error_x**2+error_y**2)

        return error

    def condition_wp(self, posx, posy):
        #Check that we have two waypoints
        while len(self.wp_list)<(self.current_i+2):
            time.sleep(0.1)
            #Wait here to at least get one waypoint

        #Check how far we are from the next waypoint
        wp_x_next = wp_list[self.current_i+1, 0]
        wp_y_next  = wp_list[self.current_i+1, 1]
        distance_from_wp = error(wp_x, posx, wp_y, posy)
       

        if distance_from_wp < self.R_treshold:
            self.current_i += 1
            while len(self.wp_list)<(self.current_i+2):
                #Wait here to at least get the next
                time.sleep(0.1)
        
        wp_nx = self.wp_list[self.current_i, 0]
        wp_ny = self.wp_list[self.current_i, 1]
        wp_n1x = self.wp_list[self.current_i+1, 0]
        wp_n1y = self.wp_list[self.current_i+1, 1]  

        conditioned_wp = np.array([posx, posy, wp_nx, wp_ny, wp_n1x, wp_n1y])
        return conditioned_wp


    #Callback functions
    def save_gotten_wps(self, wp_input):
        x = wp_input.x
        y = wp_input.y
        z = wp_input.z
        data_to_append = [x, y, z]
        self.wp_list.append(data_to_append)

    def save_gotten_pos(self, pos_input):
        #Extract position
        posx = pos_input(0)
        posy = pos_input(1)
        posz = pos_input(2)
        
        self.pos = np.array([posx, posy, posz])

        #Extract rotation
        self.quatx = pos_input(3)
        self.quaty = pos_input(4)
        self.quatz = pos_input(5)
        self.quatw = pos_input(6)

        self.quatRot = np.array([quatx, quaty, quatz, quatw])

        conditioned_wp = condition_wp(self)

        self.cond_wp_publish.publish(conditioned_wp)     

wp_conditioner = ConditionWP()
try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")   
#!/usr/bin/env python3.7
import numpy as np
import rospy
import std_msgs
import geometry_msgs



class ConditionWP:
    def __init__(self):
        self.wp_list = [[0,0]]
        self.R_treshold = 2
        self.current_i = 0


        #Publish to topic cond_wps
        self.cond_wp_publish = rospy.Publisher('cond_wps', numpy_msg(Floats))

        #Subscriber to waypoints
        self.wp_subscriber = rospy.Subscriber('wp_list', Float, callback = self.save_gotten_wps, queue_size = 10)
        #Subscriber for current position
        self.pos_subscriber = rospy.Subscriber('pos_coord', Point, save_gotten_pos)


    def save_gotten_wps(self, wp_input):
        self.wp_list.append(wp_input)
    def save_gotten_pos(self, pos_input):

        
        

    
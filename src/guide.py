#!/usr/bin/env python

import rospy 
from rospy.numpy_msg import numpy_msg
import numpy as np 


class Guidance:
    
    #Initialization function
    def __init__(self):
        #Initialize node
        rospy.init_node('guidance')

        #Start publisher for speed 
        
        #subscribe to right topics
        self.condSubscriber = rospy.Subscriber('cond_wps', numpy_msg(Floats),callback=self.calculatePsiSetpoint, queue_size=2)


    #Helping functions
    def rotation(alpha):
        c = np.cos(alpha)
        s = np.sin(alpha)


        R = np.array([[c, s],
                      [-s, c]])
        return R


    #Callback function
    def calculatePsiSetpoint(self, cond_wp):
        """Datastructure of cond_wp is
            [   current_pos_x;
                current_pos_y;
                wp_n_x;
                wp_n_y;
                wp_n_+1_x;
                wp_n_+1_y;
        """
        #Position of craft
        current_x = cond_wp[0]
        current_y = cond_wp[1]

        position = np.array([current_x, current_y])

        #Waypoint n
        wp_n_x = cond_wp[2]
        wp_n_y = cond_wp[3]
        wp_n_position = np.array([wp_n_x, wp_n_y])


        #Waypoint n+1
        wp_n1_x = cond_wp[4]
        wp_n1_y = cond_wp[5]
        wp_n1_position = np.array([wp_n1_x, wp_n1_y])
        

        #Calculating diff-vector between wp_n and wp_n+1
        wp_diff_y = wp_n2_y- wp_n_y
        wp_diff_x = wp_n1_x- wp_n_x


        #alpha, alongtrack error and cross track error
        alpha = np.arctan2(wp_diff_y, wp_diff_x)
        R_alpha = rotation(alpha)

        epsilon = R_alpha@(position-wp_n_position)
        s = epsilon[0] #along track error
        e = epsilon[1] #cross track error


        K_p = 1


        self.psi_r = np.arctan(Kp*e)

        self.psi_d = alpha + self.psi_r 



try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
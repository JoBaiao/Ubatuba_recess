#!/usr/bin/env python3

"""
Universidade Federal de Minas Gerais (UFMG) - 2023
Laboratorio CORO
Contact:
Joao Baiao, <baiaojfr.eng@gmail.com>
"""

import rospy
import tf
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from scipy.linalg import norm
import numpy as np
from Navigator_class import Navigation_class


from subprocess import Popen, PIPE
import time

class mission:
    def __init__(self):
        rospy.init_node('Navigation_node')
        self.navigator = Navigation_class()
        self.navigator.read_params()

        self.freq = 30

        #Subscribers

        # A*
        # Visao

        #Publishers

    def open_horizontallines():
        process = Popen(['python', 'horizontal_lines_ROS.py'], stdout=PIPE, stderr=PIPE)
        time.sleep(2)  
        process.terminate()

    def open_green_corner_tracker():
        process = Popen(['python', 'green_corner_tracker_ROS.py'], stdout=PIPE, stderr=PIPE)
        time.sleep(1)  
        process.terminate()

    def close_horizontallines(process):
        process.terminate()
    
        

    def run(self):
        rate = rospy.Rate(self.freq)

        # Wait a bit
        rate.sleep()

        C = [[6,1],[6,2],[5,3],[4,4],[3,4],[2,4]]

        self.navigator.set_path(C)

        while not rospy.is_shutdown():

            #print("ETAPA 3")
            #self.navigator.rotate_to_this(90)

            #print("ETAPA 1")
            #self.navigator.follow_field()

            #print("ETAPA 2")
            #self.navigator.walk_this([1,0])
            #self.navigator.walk_this([0,1])
            #self.navigator.walk_this([1,-1])


            while(True):

                time.sleep(1)

                print("ETAPA 1")
                self.navigator.walk_this([1,0])
                self.navigator.walk_this([-1,0])
                print("ETAPA 2")
                self.navigator.rotate_to_this(90)
                self.navigator.rotate_to_this(450)
                print("ETAPA 3")
                self.navigator.walk_this([0,1])
                self.navigator.walk_this([0,-1])

                lines_count_msg = rospy.wait_for_message('/terminal_content', String)
                current_value = int(lines_count_msg.data)
                if current_value==0:
                    break

                green_msg = rospy.wait_for_message('/green_corner_tracker', String, timeout=1)
                if green_msg is not None:
                    break

                break
            
                rate.sleep()

if __name__ == '__main__':
    try:
        x = mission()
        x.run()
    except rospy.ROSInterruptException:
        pass
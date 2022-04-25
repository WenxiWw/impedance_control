#!/usr/bin/env python
import rospy
import sys
import os
import pickle
from time import gmtime, strftime


from panda_msgs_srvs.msg import  robot_state_cart, robot_state_pd


class Logger:
    def __init__(self):
        fileDir = os.path.dirname(os.path.abspath(__file__))
        self.folder_path = os.path.join(fileDir, '../../../log_data')
        self.timeseries = []
        self.count = 0
        self.called = 0

    def logging_callback_pd(self, data):
        if not self.called:
            self.called = 1
            time_name = strftime("%Y-%m-%d_%H-%M", gmtime())
            self.folder_path = os.path.join(self.folder_path, 'pd', time_name)
            os.makedirs(self.folder_path)
            rospy.loginfo("Log Data to "+ self.folder_path)
        dataframe = dict(
            timestamp = data.time,
            q= data.q,
            dq=data.dq,
            q_d=data.q_d,
            error_q=data.err_q
        )
        self.timeseries.append(dataframe)
        if len(self.timeseries)==10000:
            output_file = os.path.join(self.folder_path,'data'+str(self.count)+'.pickle')
            
            with open(output_file, 'wb') as f:
                pickle.dump(self.timeseries, f)
            self.timeseries=[]
            self.count += 1
            rospy.loginfo("Dumped Data")
    
    def logging_callback_cart(self, data):
        if not self.called:
            self.called = 1
            time_name = strftime("%Y-%m-%d_%H-%M", gmtime())
            self.folder_path = os.path.join(self.folder_path, 'cart', time_name)
            os.makedirs(self.folder_path)
            rospy.loginfo("Log Data to "+ self.folder_path)
        dataframe = dict(
            timestamp = data.time,
            pos= data.position_linear,
            pos_d=data.position_linear_d,
            #ori=data.orientation,
            #ori_d=data.orientation_d,
            error_x= data.error_linear_x,
            #dq = data.dq,
            #q = data.q,
            tau_prim =data.tau_prim,
            tau_sec =data.tau_sec,
            #f_sec = data.f_sec
        )

        self.timeseries.append(dataframe)
        if len(self.timeseries)==10000:
            output_file = os.path.join(self.folder_path,'data'+str(self.count)+'.pickle')
            with open(output_file, 'wb') as f:
                pickle.dump(self.timeseries, f)
            self.timeseries=[]
            self.count += 1
            rospy.loginfo("Dumped Data")    


def log_data():
    rospy.init_node('logging_node')
    logger = Logger()
    rospy.Subscriber("robot_state_pd", robot_state_pd, logger.logging_callback_pd)
    rospy.Subscriber("robot_state_cart", robot_state_cart, logger.logging_callback_cart)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':

    log_data()

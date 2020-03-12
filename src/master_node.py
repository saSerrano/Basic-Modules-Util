#!/usr/bin/env python
# coding=utf-8
import sys
import rospy  # ROS
from basicmodutil_pkg import commBM # Basic module communication functions
import json   # JSON <--> string

# Import each data-type required by the basic module (see the input and output
# parameters of its functions)
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

class MasterNode:
    def __init__(self):
        #comm-topics
        top_comm_sub = '/function/output'
        top_comm_pub = '/master/output'

        #comm pub & sub
        self.__comm_pub = rospy.Publisher(top_comm_pub,String,queue_size=1)
        self.__comm_sub = rospy.Subscriber(top_comm_sub,String,self.__comm_cb)
        #subscriber for the user to trigger the master node
        self.__trig_sub = rospy.Subscriber('/trigger',String,self.__trig_cb)
        #DISPLAY IN CONSOLE THE BRIDGE INFO
        rospy.loginfo("MASTER NODE")
        rospy.loginfo("COMM-TOPICS")
        rospy.loginfo("\t%s",top_comm_pub)
        rospy.loginfo("\t%s",top_comm_sub)
        rospy.loginfo("######################################")

    def __comm_cb(self,data):
        #Receive the result from a function execution
        names, ros_msgs = commBM.readMsg2Ros(data.data)

        #Display the results
        rospy.loginfo("COMM_CB")
        rospy.loginfo("OUTPARAM-NAMES")
        for n in names:
            rospy.loginfo("\t%s",n)
        rospy.loginfo("OUTPARAMS")
        for m in ros_msgs:
            print(m)
        rospy.loginfo("----------------------------------")
        return

    def __trig_cb(self,data):
        if(data.data == 'A' or data.data == 'a'):
            s_msg = String('Para la funcion A')
            b_msg = Bool(False)
            ros_msg = [s_msg, b_msg]
            js = commBM.writeFunCallFromRos('DEMO_BM','functionA',ros_msg)
            msg = String(js)
            self.__comm_pub.publish(msg)
        elif(data.data == 'B' or data.data == 'b'):
            s_msg = String('Para la funcion B')
            b_msg = Bool(False)
            ros_msg = [b_msg,s_msg]
            js = commBM.writeFunCallFromRos('DEMO_BM','functionB',ros_msg)
            msg = String(js)
            self.__comm_pub.publish(msg)
        elif(str(data.data).find('p') != -1):
            if(str(data.data).find('grasp') != -1):
                t = Point(1,2,-5)
                r = Float64(0.2)
                ros_msg = [t,r]
                js = commBM.writeFunCallFromRos('PYTHON_BM','graspObject',ros_msg)
                msg = String(js)
                self.__comm_pub.publish(msg)
            elif(str(data.data).find('put') != -1):
                t = Point(6.4,9,3)
                ros_msg = [t]
                js = commBM.writeFunCallFromRos('PYTHON_BM','putObject',ros_msg)
                msg = String(js)
                self.__comm_pub.publish(msg)
            elif(str(data.data).find('safe') != -1):
                ros_msg = []
                js = commBM.writeFunCallFromRos('PYTHON_BM','goSafePos',ros_msg)
                msg = String(js)
                self.__comm_pub.publish(msg)

def main(args):
    rospy.init_node('master_node', anonymous=False)
    mn = MasterNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)

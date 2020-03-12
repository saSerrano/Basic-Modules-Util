#!/usr/bin/env python
# coding=utf-8
import sys
import rospy  # ROS
from basicmodutil_pkg import commBM # Basic module communication functions
import json   # JSON <--> string
from std_msgs.msg import String # Necessary for the basic-mod comunicates with the master node

from std_msgs.msg import Bool # For success flags
from geometry_msgs.msg import Point # For the end-effector positions
from std_msgs.msg import Float64 # For object's radio

class BasicModTemplate:
    def __init__(self,bm_name):
        #Name of the basic module this node will serve as bridge
        self.__basic_mod_name = bm_name

        #comm-topics
        top_comm_pub = "/function/output"
        top_comm_sub = "/master/output"

        #comm pub & sub
        self.__comm_pub = rospy.Publisher(top_comm_pub,String,queue_size=1)
        self.__comm_sub = rospy.Subscriber(top_comm_sub,String,self.__comm_cb)

        #DISPLAY IN CONSOLE THE BRIDGE INFO
        rospy.loginfo("BASIC-MODULE")
        rospy.loginfo("\t%s",bm_name)
        rospy.loginfo("COMM-TOPICS")
        rospy.loginfo("\t%s",top_comm_pub)
        rospy.loginfo("\t%s",top_comm_sub)
        rospy.loginfo("######################################")

    def __comm_cb(self,data):
        #Parse the function-invocation message into a dictionary
        bm, func, msgs = commBM.readFunCall(data.data)
        #Debug info
        rospy.loginfo('COMM_CB')
        rospy.loginfo('BASIC MODULE')
        rospy.loginfo("\t%s",bm)
        rospy.loginfo('FUNCTION')
        rospy.loginfo("\t%s",func)
        rospy.loginfo('DATA')
        print(msgs)
        rospy.loginfo("----------------------------------")

        #Check if this basic module is being requested
        if(bm == self.__basic_mod_name):
            if(func == 'graspObject'):
                self.__graspObject(msgs)
            elif(func == 'putObject'):
                self.__putObject(msgs)
            elif(func == 'goSafePos'):
                self.__goSafePos(msgs)
            #...add as many cases as functions the basic module has
        return

    def __graspObject(self,msgs):
        target_pos = msgs[0]
        obj_radius = msgs[1]

        #Do something with the input-params

        #Return the output-params
        success = Bool(False)
        final_pos = Point(0,1.2,2.3)
        out_params = [success, final_pos]
        names = ['success', 'final_pos']
        msg_str = commBM.writeMsgFromRos(out_params, names)
        out_msg = String(msg_str)
        self.__comm_pub.publish(out_msg)

        #Debug info
        rospy.loginfo('EXECUTING FUNCTION')
        rospy.loginfo("\tgraspObject")
        rospy.loginfo('OUTPARAM NAMES')
        for n in names:
            rospy.loginfo('\t%s',n)
        rospy.loginfo('OUTPARAMS')
        print(out_params)
        rospy.loginfo("----------------------------------")
        return

    def __putObject(self,msgs):
        target_pos = msgs[0]

        #Do something with the input-params

        #Return the output-params
        success = Bool(True)
        final_pos = target_pos
        out_params = [success, final_pos]
        names = ['success', 'final_pos']
        msg_str = commBM.writeMsgFromRos(out_params, names)
        out_msg = String(msg_str)
        self.__comm_pub.publish(out_msg)

        #Debug info
        rospy.loginfo('EXECUTING FUNCTION')
        rospy.loginfo("\tputObject")
        rospy.loginfo('OUTPARAM NAMES')
        for n in names:
            rospy.loginfo('\t%s',n)
        rospy.loginfo('OUTPARAMS')
        print(out_params)
        rospy.loginfo("----------------------------------")
        return

    def __goSafePos(self,msgs):

        #Attempt to take the robotic arm to the home position

        #Return the output-params
        success = Bool(True)
        final_pos = Point(0,0,0)
        out_params = [success, final_pos]
        names = ['success', 'final_pos']
        msg_str = commBM.writeMsgFromRos(out_params, names)
        out_msg = String(msg_str)
        self.__comm_pub.publish(out_msg)

        #Debug info
        rospy.loginfo('EXECUTING FUNCTION')
        rospy.loginfo("\tgoSafePos")
        rospy.loginfo('OUTPARAM NAMES')
        for n in names:
            rospy.loginfo('\t%s',n)
        rospy.loginfo('OUTPARAMS')
        print(out_params)
        rospy.loginfo("----------------------------------")
        return

def main(args):
	basic_module_name = 'PYTHON_BM'
	rospy.init_node(basic_module_name, anonymous=False)
	BasicModTemplate(basic_module_name)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)

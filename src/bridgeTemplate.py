#!/usr/bin/env python
# coding=utf-8
import sys
import rospy  # ROS
import commBM # Basic module communication functions
import json   # JSON <--> string

# Import each data-type required by the basic module (see the input and output
# parameters of its functions)
from std_msgs.msg import String
from std_msgs.msg import Bool

class CppPythonBridge:
	def __init__(self,bm_name):
		#Name of the basic module this node will serve as bridge
		self.__basic_mod_name = bm_name

		#comm-topics
		top_comm_pub = "/function/output"
		top_comm_sub = "/master/output"

		#sync-topics
		top_sync_pub = "/"+bm_name+"/BM_input"
		top_sync_sub = "/"+bm_name+"/BM_output"

		#data-topics
		self.__top_data_string = "/"+bm_name+"/string"
		self.__top_data_bool = "/"+bm_name+"/bool"
		# ...

		#-----------------------------------------------------------------------

		#comm pub & sub
		self.__comm_pub = rospy.Publisher(top_comm_pub,String,queue_size=1)
		self.__comm_sub = rospy.Subscriber(top_comm_sub,String,self.__comm_cb)

		#sync pub & sub
		self.__sync_pub = rospy.Publisher(top_sync_pub,String,queue_size=1)
		self.__sync_sub = rospy.Subscriber(top_sync_sub,String,self.__sync_cb)

		self.__topics = {} # STORE DATA-TOPICS IN A DICTIONARY (key=msg-type,value=topic)
		self.__pubs = {}   # STORE DATA-PUBS IN A DICTIONARY (key=msg-type,value=pub)

		#data pubs & subs
		tmp = commBM.getRosType(String())
		self.__topics[tmp] = self.__top_data_string
		self.__pubs[tmp] = rospy.Publisher(self.__top_data_string,String,queue_size=1)
		self.__string_sub = rospy.Subscriber(self.__top_data_string,String,self.__string_cb)

		tmp = commBM.getRosType(Bool())
		self.__topics[tmp] = self.__top_data_bool
		self.__pubs[tmp] = rospy.Publisher(self.__top_data_bool,Bool,queue_size=1)
		self.__bool_sub = rospy.Subscriber(self.__top_data_bool,Bool,self.__bool_cb)

		# ...
		#Add a pair of pub & sub for each data-topic between this node and the
		#basic module node

		#-----------------------------------------------------------------------

		#A dictionary will serve as a receiving data buffer, that must have an
		#attribute for each data-topic. The data-topics are employed as keys
		#in the dictionary.
		self.__buffer = {}
		self.__buffer[self.__top_data_string] = String()
		self.__buffer[self.__top_data_bool] = Bool()
		# ...

		#DISPLAY IN CONSOLE THE BRIDGE INFO
		rospy.loginfo("BRIDGE FOR BASIC-MODULE %s",bm_name)
		rospy.loginfo("COMM-TOPICS")
		rospy.loginfo("\t%s",top_comm_pub)
		rospy.loginfo("\t%s",top_comm_sub)
		rospy.loginfo("SYNC-TOPICS")
		rospy.loginfo("\t%s",top_sync_pub)
		rospy.loginfo("\t%s",top_sync_sub)
		rospy.loginfo("DATA-TOPICS")
		rospy.loginfo("\t%s",self.__top_data_string)
		rospy.loginfo("\t%s",self.__top_data_bool)
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
			tmp = {}
			tmp["function"] = func
			tmp["inparam_topics"] = []
			for m in msgs:
				rt = commBM.getRosType(m)
				#Get the data topic
				tmp["inparam_topics"].append(self.__topics[rt])
				#Publish the msg
				self.__pubs[rt].publish(m)

			#Make a delay
			rospy.sleep(1.0)

			#Publish in the sync-topic the requested function and the data-topics
			#in which the input-parameters for that function were published
			js = json.dumps(tmp)
			sync_msg = String(js)
			self.__sync_pub.publish(sync_msg)
		return

	def __sync_cb(self,data):
		#Get the output-params of the function that finished its execution
		js = data.data
		tmp = commBM.json_loads_byteified(js)
		outparam_names = tmp["outparam_names"]
		outparam_topics = tmp["outparam_topics"]
		outparams = []
		for ot in outparam_topics:
			outparams.append(self.__buffer[ot])

		#Debug info
		rospy.loginfo("SYNC_CB")
		rospy.loginfo("OUTPARAM-NAMES")
		for n in outparam_names:
			rospy.loginfo("\t%s",n)
		rospy.loginfo("OUTPARAM-TOPIC")
		for t in outparam_topics:
			rospy.loginfo("\t%s",t)
		rospy.loginfo('----------------------------------')

		#Build the message to be published to the blackboard and master
		msg = commBM.writeMsgFromRos(outparams,outparam_names)
		comm_msg = String(msg)
		self.__comm_pub.publish(comm_msg)
		return

	#Data callbacks only have to store the received data in the buffer in its
	#respective attribute
	def __string_cb(self,data):
		self.__buffer[self.__top_data_string] = data
		return

	def __bool_cb(self,data):
		self.__buffer[self.__top_data_bool] = data
		return

def main(args):
	basic_module_name = 'DEMO_BM'
	rospy.init_node(basic_module_name+'_bridge', anonymous=True)
	cpb = CppPythonBridge(basic_module_name)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)

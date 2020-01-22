#Description: This script helps to write the JSON description of basic modules
#that are employed to develop skills for in the MARKOVITO team.
#
#Author: Sergio A. Serrano
#
#Email: sserrano@inaoep.mx
#
#Date: 10/Jan/2020

import sys
import json

def bmInfo():
	#Display the main structure of a basic module to the user
	print(">> Structure of a basic module")
	print("-----------------------------------------------------------------------")
	print("\tBasic Module: \"...\", <The basic module's name>")
	print("\tBrief Description: \"...\", <A short text of what the BM does>")
	print("\tHardware Required: <A list of hardware components required by th BM>")
	print("\t[")
	print("\t\t\"...\"")
	print("\t],")
	print("\tFunctions: <List of functions the module can execute>")
	print("\t[")
	print("\t\tName: \"...\", <The function's name>")
	print("\t\tExec-condition: \"...\", <Description of conditions that must be met before the function can be executed>")
	print("\t\tDescription: \"...\", <Brief description of what the function does>")
	print("\t\tInput-params: <List of parameters the function receives and needs to perform its task>")
	print("\t\t[")
	print("\t\t\t{")
	print("\t\t\t\tName: \"...\", <The parameter's name>")
	print("\t\t\t\tData-type: \"...\", <Type of data that the parameter receives as value>")
	print("\t\t\t\tDescription: \"...\", <Description of the parameter>")
	print("\t\t\t},")
	print("\t\t\t\"...\"")
	print("\t\t],")
	print("\t\tOutput-params: <List of parameters the function returns as result of executing the function>")
	print("\t\t[")
	print("\t\t\t{")
	print("\t\t\t\tName: \"...\", <The parameter's name>")
	print("\t\t\t\tData-type: \"...\", <Type of data that the parameter returns>")
	print("\t\t\t\tDescription: \"...\", <Description of the parameter>")
	print("\t\t\t},")
	print("\t\t\t\"...\"")
	print("\t\t]")
	print("\t]")
	print("-----------------------------------------------------------------------")

def bmRead():
	#Create a dictionary for the JSON structure
	js = {}

	#Add values to the JSON
	print(">> Submit the requested attributes")
	bm = raw_input("\tBasic Module: ")
	js["Basic Module"] = bm
	bd = raw_input("\tBrief Description: ")
	js["Brief Description"] = bd
	print("\n>> Submit the list of hardware components required and submit # when done")
	hr = []
	while(True):
		tmp = raw_input("\tHardware-req-"+str(len(hr))+": ")
		if(tmp == "#"):
			break
		else:
			hr.append(tmp)
	js["Hardware Required"] = hr

	#Gather the list of functions
	print("\n>> Submit the list of functions and submit # when done")
	fun_vec =[]
	while(True):
		fun = {}
		print("\t*******************")
		n = raw_input("\tFunction-Name: ")
		if(n == "#"):
			break
		fun["Name"] = n
		ec = raw_input("\tExec-condition: ")
		if(ec == "#"):
			break
		fun["Exec-condition"] = ec
		d = raw_input("\tDescription: ")
		if(d == "#"):
			break
		fun["Description"] = d
		print("\n\t>> Submit the list of input-params and submit # when done")
		ip_vec = []
		while(True):
			fn = raw_input("\t\tParam-Name: ")
			if(fn == "#"):
				break
			dt = raw_input("\t\tData-type: ")
			if(dt == "#"):
				break
			fd = raw_input("\t\tDescription: ")
			if(fd == "#"):
				break
			ip = {}
			ip["Name"] = fn
			ip["Data-type"] = dt
			ip["Description"] = fd
			ip_vec.append(ip)
			print("\t\t-------------------")
		fun["Input-params"] = ip_vec
		print("\n\t>> Submit the list of output-params and submit # when done")
		op_vec = []
		while(True):
			fn = raw_input("\t\tName: ")
			if(fn == "#"):
				break
			dt = raw_input("\t\tData-type: ")
			if(dt == "#"):
				break
			fd = raw_input("\t\tDescription: ")
			if(fd == "#"):
				break
			op = {}
			op["Name"] = fn
			op["Data-type"] = dt
			op["Description"] = fd
			op_vec.append(op)
			print("\t\t-------------------")
		fun["Output-params"] = op_vec
		fun_vec.append(fun)
	js["Functions"] = fun_vec

	return js

def main(argv):
	#Check if a file name was provided
	if(len(argv) != 2):
		print("Usage: python writeBM.py <Name of JSON output file>")
		print("For info: python writeBM.py -h")
		return
	elif(argv[1] == "-h" or argv[1] == "-help"):
		#Display info of the BM's structure
		bmInfo()
		return

	#Attempt to create the new file
	try:
		#Create the JSON output file
		outfile = open(argv[1],"w")

		#Gather the BM data from the console
		bm = bmRead()

		#Save the BM in la JSON file
		bm_str = json.dumps(bm,sort_keys=True,indent=4)
		outfile.write(bm_str)

		#Close the JSON file
		outfile.close()
	except IOError:
		print("Could not create file " + str(argv[1]))
		return

if __name__ == "__main__":
	main(sys.argv)

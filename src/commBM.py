#!/usr/bin/env python
# coding=utf-8
from rospy_message_converter import json_message_converter
import json

#Functions that convert the unicode data from JSON into string ojects. These
#were taken from the accepted answer in:
#https://stackoverflow.com/questions/956867/how-to-get-string-objects-instead-of-unicode-from-json
def json_load_byteified(file_handle):
    return _byteify(
        json.load(file_handle, object_hook=_byteify),
        ignore_dicts=True)
def json_loads_byteified(json_text):
    return _byteify(
        json.loads(json_text, object_hook=_byteify),
        ignore_dicts=True)
def _byteify(data, ignore_dicts = False):
    # if this is a unicode string, return its string representation
    if isinstance(data, unicode):
        return data.encode('utf-8')
    # if this is a list of values, return list of byteified values
    if isinstance(data, list):
        return [ _byteify(item, ignore_dicts=True) for item in data ]
    # if this is a dictionary, return dictionary of byteified keys and values
    # but only if we haven't already byteified it
    if isinstance(data, dict) and not ignore_dicts:
        return {
            _byteify(key, ignore_dicts=True): _byteify(value, ignore_dicts=True)
            for key, value in data.iteritems()
        }
    # if it's anything else, return it in its original form
    return data

def getRosType(msg):
    """
    This function takes a ROS message object and generates, in the form of a
    string, its message type.

    Parameters
    ----------
    msg : ROS message
        ROS message object whose type is required.

    Returns
    -------
    msg_type : str
        The type of the ROS message in 'msg'.
    """

    #Check that msg is a ROS message instance
    tmp = str(type(msg))
    tkn = tmp.split('.msg._')
    if(len(tkn) != 2):
        return ''

    #Get the ROS message type
    i = tkn[0].find("'")
    pkg = tkn[0][i+1:]
    tmp = tkn[1].split('.')
    msg = tmp[0]
    msg_type = pkg + '/' + msg

    return msg_type

def writeVarRequest(vars):
    """
    This function writes a request for the values of a list of variables.

    Parameters
    ----------
    vars : list
        List of the names (str) of those variables that are being requeusted.

    Returns
    -------
    str
        The request in the form of a JSON string.
    """

    #check that vars is a list
    if(not isinstance(vars,list)):
        return ''

    #check that every element in vars is a string
    for v in vars:
        if(not isinstance(v,str)):
            return ''

    #Build the request string
    tmp = {}
    tmp["var_names"] = []
    for v in vars:
        tmp["var_names"].append(v)
    req_str = json.dumps(tmp)

    return req_str

def readVarRequest(req):
    """
    This function takes a request string written by 'writeVarRequest' and
    returns the list of variables that are being requested.

    Parameters
    ----------
    req : str
        Request in the form of a JSON string.

    Returns
    -------
    list
        List of the requested variable names (str).
    """

    #check that req is a string
    if(not isinstance(req,str)):
        return []

    #Convert the json string into a dictionary
    vars = []
    try:
        tmp = json_loads_byteified(req)
        vars = tmp["var_names"]
    except:
        return []

    return vars

def writeMsgFromRos(values,names):
    """
    This function writes a list of ROS messages into a JSON string as a single
    message.

    Parameters
    ----------
    values : list
        List of ROS message objects.
    names : list
        List of names (str) of the ROS messages in 'values'.

    Returns
    -------
    str
        A deserialized JSON that holds the ROS messages in 'values', their
        variable names and ROS message types.
    """

    #check that vals & names are lists with the same length
    if(not isinstance(values,list)):
        return ''
    if(not isinstance(names,list)):
        return ''
    if(len(values) != len(names)):
        return ''

    #Check that every element in names is a string
    for n in names:
        if(not isinstance(n,str)):
            return ''

    #Convert every element in vals into a json string and get its ROS-type
    js = []
    mt = []
    for v in values:
        tmp = str(type(v))
        #Get the ros-msg type
        msg_type = getRosType(v)
        #Check if v is a ROS message
        if(msg_type == ''):
            return ''
        mt.append(msg_type)
        #Convert ros2json
        tmp = json_message_converter.convert_ros_message_to_json(v)
        js.append(tmp)

    #Put lists 'names', 'mt' and 'js' into a dictionary and convert into a JSON string
    tmp = {}
    tmp["names"] = names
    tmp["rostypes"] = mt
    tmp["msgs"] = js
    json_str = json.dumps(tmp)

    return json_str

def writeMsgFromJson(values,names,msgtypes):
    """
    This function writes a list of ROS messages into a JSON string as a single
    message.

    Parameters
    ----------
    values : list
        List of ROS messages in the form of strings (deserialized JSON).
    names : list
        List of names (str) of the ROS messages in 'values'.
    msgtypes : list
        List of types (str) of the ROS messages in 'values'.

    Returns
    -------
    str
        A deserialized JSON that holds the ROS messages in 'values', their
        variable names and ROS message types.
    """

    #check that values, names & msgtypes are lists with the same length
    if(not isinstance(values,list)):
        return ''
    if(not isinstance(names,list)):
        return ''
    if(not isinstance(msgtypes,list)):
        return ''
    if(len(values) != len(names)):
        return ''
    if(len(values) != len(msgtypes)):
        return ''

    #Check that every element in values, names & msgtypes is a string
    for i in range(len(values)):
        if(not isinstance(msgtypes[i],str)):
            return ''
        if(not isinstance(names[i],str)):
            return ''
        if(not isinstance(values[i],str)):
            return ''

    #Put lists 'values', 'names' and 'msgtypes' into a dictionary and convert into a JSON string
    tmp = {}
    tmp["names"] = names
    tmp["rostypes"] = msgtypes
    tmp["msgs"] = values
    json_str = json.dumps(tmp)

    return json_str

def readMsg2Ros(msg):
    """
    This function reads a JSON string written by 'writeMsgFromRos' or
    'writeMsgFromJson' and transforms it into a list of ROS message objects
    and a list of their variable names.

    Parameters
    ----------
    msg : str
        Deserialized JSON that holds a list of ROS messages.

    Returns
    -------
    names : list
        Variable names for the ROS objects in 'ros_msgs'.
    ros_msgs : list
        ROS message objects.
    """

    #Check that msg is a string
    if(not isinstance(msg,str)):
        return []

    #deserialize the json string
    tmp = {}
    try:
        tmp = json_loads_byteified(msg)
    except:
        return []

    #Get the 'names', 'msgs' and 'rostypes' lists
    names = tmp["names"]
    msgs = tmp["msgs"]
    rostypes = tmp["rostypes"]

    #Convert each string in 'msgs' into a ROS message instance
    ros_msgs = []
    for i in range(len(rostypes)):
        rm = json_message_converter.convert_json_to_ros_message(rostypes[i], msgs[i])
        ros_msgs.append(rm)

    return names, ros_msgs

def readMsg2Json(msg):
    """
    This function reads a JSON string written by 'writeMsgFromRos' or
    'writeMsgFromJson' and transforms it into a list of ROS messages in the form
    of deserialized JSON strings.

    Parameters
    ----------
    msg : str
        Deserialized JSON that holds a list of ROS messages.

    Returns
    -------
    names : list
        Variable names for the ROS objects in 'ros_msgs'.
    msgs : list
        Deserialized JSONs that hold each a ROS message.
    rostypes : list
        ROS types (str) of each ROS message in 'msgs'
    """

    #Check that msg is a string
    if(not isinstance(msg,str)):
        return []

    #deserialize the json string
    tmp = {}
    try:
        tmp = json_loads_byteified(msg)
    except:
        return []

    #Get the 'names', 'rostypes' and 'msgs' lists
    names = tmp["names"]
    msgs = tmp["msgs"]
    rostypes = tmp["rostypes"]

    #Return lists of variable-names, ROS messages & ROS-msg-types
    return names, msgs, rostypes

def writeFunCallFromRos(basicmod,fun,msgs):
    """
    This function writes an message to invoke function 'fun' contained in the
    basic module 'basicmod'. To invoke function 'fun', the ROS messages in
    'msgs' are passed as input parameters.

    Parameters
    ----------
    basicmod : str
        Name of the basic module whose function 'fun' will be invoked.
    fun : str
        Name of the function that will be invoked.
    msgs : list
        List of ROS message objects that the function 'fun' requires as input
        parameters to work.

    Returns
    -------
    json_str : str
        Deserialized JSON that specifies a function to be invoked as well as
        the input parameters required.
    """

    #Check that basicmod & fun are strings
    if(not isinstance(basicmod,str) or not isinstance(fun,str)):
        return ''
    if(not isinstance(msgs,list)):
        return ''

    #Convert every element in vals into a json string and get its ROS-type
    js = []
    mt = []
    for m in msgs:
        #Get the ROS message type
        msg_type = getRosType(m)
        #Check if m is a ROS message
        if(msg_type == ''):
            return ''
        mt.append(msg_type)
        #Convert ros2json
        tmp = json_message_converter.convert_ros_message_to_json(m)
        js.append(tmp)

    #Build the json object for calling the function & serialize it into a string
    tmp = {}
    tmp["basic_mod"] = basicmod
    tmp["function"] = fun
    tmp["msgs"] = js
    tmp["rostypes"] = mt
    json_str = json.dumps(tmp,indent=2)

    return json_str

def writeFunCallFromJson(basicmod,fun,msgs,msgtypes):
    """
    This function writes an message to invoke function 'fun' contained in the
    basic module 'basicmod'. To invoke function 'fun', the ROS messages in
    'msgs' are passed as input parameters.

    Parameters
    ----------
    basicmod : str
        Name of the basic module whose function 'fun' will be invoked.
    fun : str
        Name of the function that will be invoked.
    msgs : list
        List of deserialized JSONs of the ROS messages that the function 'fun'
        requires as input parameters to work.
    msgtypes : list
        List of ROS message types of those messages in 'msgs'.

    Returns
    -------
    json_str : str
        Deserialized JSON that specifies a function to be invoked as well as
        the input parameters required.
    """

    #Check that basicmod & fun are strings
    if(not isinstance(basicmod,str) or not isinstance(fun,str)):
        return ''
    #Check that msgs & msgtypes are lists
    if(not isinstance(msgs,list) or not isinstance(msgtypes,list)):
        return ''

    #Check that msgs & msgtypes have the same lenght
    if(len(msgs) != len(msgtypes)):
        return ''

    #Check that every element in msgs & msgtypes is a string
    for i in range(len(msgs)):
        if(not isinstance(msgs[i],str) or not isinstance(msgtypes[i],str)):
            return ''

    #Build the json object for calling the function & serialize it into a string
    tmp = {}
    tmp["basic_mod"] = basicmod
    tmp["function"] = fun
    tmp["msgs"] = msgs
    tmp["rostypes"] = msgtypes
    json_str = json.dumps(tmp)

    return json_str

def readFunCall(funcall):
    """
    This function reads a JSON string written by 'writeFunCallFromRos' or
    'writeFunCallFromJson' and transforms it into a basic-module name, a
    function name and a list of ROS message objects.

    Parameters
    ----------
    funcall : str
        Deserialized JSON that holds a function invokation request.

    Returns
    -------
    bm : str
        Name of the basic module whose function is being requested.
    func : str
        Name of the function that is being requested.
    msgs : list
        List of ROS message objects that will serve as input parameters for
        the requested function.
    """

    #check that funcall is a string
    if(not isinstance(funcall, str)):
        return []

    #deserialize the json string
    tmp = {}
    try:
        tmp = json_loads_byteified(funcall)
    except:
        return []

    #Get the JSON's attributes
    bm = tmp["basic_mod"]
    func = tmp["function"]
    js = tmp["msgs"]
    rt = tmp["rostypes"]

    #Convert the json strings into ROS messages
    msgs = []
    for i in range(len(js)):
        msg = json_message_converter.convert_json_to_ros_message(rt[i],js[i])
        msgs.append(msg)

    return bm, func, msgs

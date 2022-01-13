#!/usr/bin/env python3

import rospy
import socket
import json
import struct
import threading
import copy
from std_msgs.msg import Int32MultiArray,Bool,String
from hyundai_v2x_msgs.msg import spatData
from hyundai_v2x_msgs.msg import stateData


PVD_type=0x01 #1hz
SPat_type=0x61 #10hz
RSA_type=0x62 #2hz
TIM_type=0x63 #1hz
MAP_type=0x64 #10hz
RTCM_type=0x65 #10hz 
BSM_type=0x66 #10hz

ip = "127.0.0.1" # "127.0.0.1" 
port = 9100   # 5G 9100
clientSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

buffer_size=57665
signal_group_id=0
clientSocket.connect((ip,port))				
print("connected.")
global struct_data
global heading
struct_data=b''
heading=0
big_loop=True # global variable
start_456 = False

file_name='pvd_example.json'
idx = 0
with open(file_name,'r') as st_json:
    read_dict=json.load(st_json)

def sendPVD():
    global struct_data
    # print(struct_data)
    clientSocket.sendall(struct_data)
    threading.daemon = True 
    threading.Timer(1,sendPVD).start()

sendPVD()

def pvd_cb(msg):
    global struct_data
    global heading
    read_dict["snapshots"][0]["thePosition"]["long"]=msg.data[0]
    read_dict["snapshots"][0]["thePosition"]["lat"]=msg.data[1]   
    read_dict["snapshots"][0]["thePosition"]["heading"]=msg.data[2]
    heading=int(msg.data[2]*0.0125)
    read_data=json.dumps(read_dict,ensure_ascii=False)
    data_size = (len(json.dumps(read_dict).encode('utf-8')))
    header=struct.pack('!IB',(data_size+5),1)
    struct_data =header+bytes(read_data,encoding="utf-8")

def lab_cb(msg):
    global big_loop
    big_loop = msg.data

def stratline_cb(msg):
    global start_456
    start_line=msg.data
    if start_line =="BigLoopFrom456":
        start_456=True
    elif start_line =="BigLoopFrom123": 
        start_456=False
    else:
        print("check tcp_client_si.py ")
   

rospy.init_node('hyundai_v2x')
rospy.Subscriber('lon_lat_heading',Int32MultiArray,pvd_cb)   
rospy.Subscriber('/route_planner/big_loop',Bool,lab_cb) 
rospy.Subscriber('/route_planner/big_loop_type',String,stratline_cb) 

spat_pub=rospy.Publisher('/light_state',spatData,queue_size=1)
# map_pub=rospy.Publisher('/map_data',String,queue_size=1)
trafficLightDataPrev=spatData()
while not rospy.is_shutdown():

    trafficLightData=spatData()
    trafficLightSignalData=stateData()
    raw_data=clientSocket.recv(buffer_size)

    restored_data =struct.unpack('!IB', raw_data[:5])
    data_length=restored_data[0]
    data_type=restored_data[1]
    if(data_type==0x61):
        json_data=raw_data[5:(data_length+5)]
        try:
            jdata=json.loads(json_data)
        except ValueError as e:
            print("json value error ")
            # raise KeyError from e     
        else:
            
            trafficLightData.intersectionName=jdata["intersections"][0]["name"]
            trafficLightData.intersectionId=jdata["intersections"][0]["id"]["id"]

            for i in range(len(jdata["intersections"][0]["states"])):

                if 215 <heading < 235 :

                    signal_group_id=50
                elif 305 <heading < 325 :
                    signal_group_id=60

                elif 35 <heading < 55:
                    signal_group_id=70

                elif 120 <heading < 145:
                    signal_group_id=80
                else:
                    print("exception")    
                trafficLightSignalData.movementName=jdata["intersections"][0]["states"][i]["movementName"]
                trafficLightSignalData.signalGroupID=jdata["intersections"][0]["states"][i]["signalGroup"]
                trafficLightSignalData.eventState=jdata["intersections"][0]["states"][i]["state-time-speed"][0]["eventState"]
                trafficLightSignalData.minEndTime=jdata["intersections"][0]["states"][i]["state-time-speed"][0]["timing"]["minEndTime"]
                        
                if trafficLightData.intersectionId==3185:   # "Sangam MBC"
                    if  trafficLightSignalData.signalGroupID== 80:
                        trafficLightData.signalGroupId= 80
                        trafficLightData.seePediSignal=True
                        if(trafficLightSignalData.movementName=="PED"):
                            trafficLightData.states.append(trafficLightSignalData) 
                            trafficLightSignalData=stateData()
                        
                elif  trafficLightData.intersectionId==3608:  #  "World Cup Park 2 Complex"
                    if  trafficLightSignalData.signalGroupID== 60:
                        trafficLightData.signalGroupId= 60
                        trafficLightData.seePediSignal=True
                        if(trafficLightSignalData.movementName=="PED"):
                            trafficLightData.states.append(trafficLightSignalData) 
                            trafficLightSignalData=stateData()
                
                elif  trafficLightData.intersectionId==3602:  #  "World Cup Park 4 Complex"
                    if  trafficLightSignalData.signalGroupID== 70:
                        trafficLightData.signalGroupId= 70
                        trafficLightData.seePediSignal=True
                        if(trafficLightSignalData.movementName=="PED"):
                            trafficLightData.states.append(trafficLightSignalData) 
                            trafficLightSignalData=stateData()
                           
                elif  trafficLightData.intersectionId==3599:  #  "World Cup Park 5 Complex"
                    if  trafficLightSignalData.signalGroupID== 70:
                        trafficLightData.signalGroupId= 70
                        trafficLightData.seePediSignal=True
                        if(trafficLightSignalData.movementName=="PED"):
                            trafficLightData.states.append(trafficLightSignalData) 
                            trafficLightSignalData=stateData()
          
                elif trafficLightData.intersectionId==3610 and big_loop==False:  # "Sangam Elementary School"                
                    if trafficLightSignalData.signalGroupID== 60:
                        trafficLightData.signalGroupId= 60
                        trafficLightData.seePediSignal=True
                        if(trafficLightSignalData.movementName=="PED"):
                            trafficLightData.states.append(trafficLightSignalData) 
                            trafficLightSignalData=stateData()


                elif trafficLightData.intersectionId==3600 and big_loop==True:   # "DMC"       
                             
                    if trafficLightSignalData.signalGroupID== 70:
                        trafficLightData.signalGroupId= 70
                        trafficLightData.seePediSignal=False
                        if(trafficLightSignalData.movementName=="LEFT"):
                            trafficLightData.states.append(trafficLightSignalData) 
                            trafficLightSignalData=stateData()

            
                elif trafficLightData.intersectionId==3610 and big_loop==True and start_456==True :  # "Sangam Elementary School_456"                
                    if trafficLightSignalData.signalGroupID== 50:
                        trafficLightData.signalGroupId= 50
                        trafficLightData.seePediSignal=True
                        if(trafficLightSignalData.movementName=="PED"):
                            trafficLightData.states.append(trafficLightSignalData) 
                            trafficLightSignalData=stateData()
                            
                else:                     
                    if trafficLightSignalData.signalGroupID == signal_group_id:
                        trafficLightData.signalGroupId= signal_group_id
                        trafficLightData.seePediSignal=False
                        trafficLightData.states.append(trafficLightSignalData) 
                        trafficLightSignalData=stateData()
                      
                if(idx > 1 and len(trafficLightData.states) != 0):
                    spat_pub.publish(trafficLightData)
                    
                else:
                    print(len(trafficLightDataPrev.states))
                    spat_pub.publish(trafficLightDataPrev)

                if len(trafficLightData.states) != 0:
                    trafficLightDataPrev = copy.copy(trafficLightData)
                    
                idx = idx + 1
        
                if idx > 60:
                    idx = 2
 
clientSocket.close()			
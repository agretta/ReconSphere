#!/usr/bin/python3
import time
import socket
import threading

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def unit_vector(vector):
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

# spawn a new thread to wait for input 
def get_user_input(user_input_ref):
    while user_input_ref[0] != 'end':
        user_input_ref[0] = input("Type 'end' to stop scan: ")


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# ax.scatter(x,y,z)

print("Initializing ReconSphere Data Server")
serversocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
port = 8347
serversocket.bind(('', port))

arduino_ip = "192.168.43.251"
arduino_port = 2390

raw_data = []
init_gravity = []

data_points = []
sensor_pos = [[0,0,1],[0,0,-1],[1,0,0],[-1,0,0]]

user_input = [None]
while True:
    start = ''
    start = input("Enter start to begin scan: ")
    if start == 'start':

        # start the thread looking for the end input
        mythread = threading.Thread(target=get_user_input, args=(user_input,))
        mythread.daemon = True
        mythread.start()

        run = True
        s = 'start';
        # TODO: check encoding on both sides
        serversocket.sendto(s.encode("utf8"), (arduino_ip , arduino_port));

        # get the init gravity reading
        msg, addr = serversocket.recvfrom(1024)
        # msg = "111|222|333"
        msg = msg.decode("ascii");
        print(msg)
        decoded_msg = msg.split('|')
        init_gravity = decoded_msg
        print(init_gravity)

        # recieve distance data from the ReconSphere, and plot it
        while run:
            msg, addr = serversocket.recvfrom(1024)
            # decoded_msg = msg.decode("utf8");
            # msg = "111|222|333|1234|2345|3456|4567"
            msg = msg.decode("ascii");
            decoded_msg = msg.split('|')
            print(decoded_msg);
            print(msg);
            raw_data.append([ decoded_msg[0:3] , decoded_msg[3] , decoded_msg[4] , decoded_msg[5] , decoded_msg[6] ]);

            #########################################
            # TODO: Do the graphing thing here, and fancy math
            #########################################

            raw_data[0]
            angle = angle_between(init_gravity, raw_data[-1][0])
            rot = [[1,0,0],[0,np.cos(angle),-np.sin(angle)],[0,np.sin(angle),np.cos(angle)]]

            for i in range(0,4):
                sensor = sensor_pos[i]
                sen_dist = sensor * raw_data[-1][i+1]
                print(sen_dist)
                new_sensor_pos = np.multiply(sen_dist, rot)
                print(new_sensor_pos)
                new_sensor_pos[1] += 10
                data_points.append(new_sensor_pos)

            ax.scatter(data_points[:,0],y[:,1],z[:,2])

            # maybe if the ball detects that its done, or erred, have it end here
            if msg == 'end':
                run = False;
                print("AN ERROR OCCURED")
            # user typed end, so stop the server 
            if user_input[0] == 'end':
                user_input[0] = None
                run = False
                s = "end"
                serversocket.sendto(s.encode("utf8"), (arduino_ip , arduino_port));
                print("Halting ReconSphere...")
                time.sleep(2)


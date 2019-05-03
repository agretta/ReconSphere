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

prev_loc = 0
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
sensor_pos = [[-1,0,0],[1,0,0],[1,0,0],[-1,0,0]]

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
        msg = msg.decode("utf8");
        decoded_msg = msg.split('|')
        decoded_msg = [int(i.rstrip('\x00').rstrip(' ')) for i in decoded_msg]
        init_gravity = decoded_msg
        plt.ion()
        # recieve distance data from the ReconSphere, and plot it
        while run:
            msg, addr = serversocket.recvfrom(1024)
            msg = msg.decode("ascii");
            decoded_msg = msg.split('|')
            decoded_msg = [int(i.rstrip('\x00').rstrip(' ')) for i in decoded_msg]
            raw_data.append([ decoded_msg[0:3] , int(decoded_msg[3]) , int(decoded_msg[4]) , int(decoded_msg[5]) , int(decoded_msg[6]) ]);
            print(raw_data[-1])

            angle = angle_between(init_gravity, raw_data[-1][0])
            rot = [[1,0,0],[0,np.cos(angle),-np.sin(angle)],[0,np.sin(angle),np.cos(angle)]]

            for i in range(0,2):
                # print(rot)
                sensor = sensor_pos[i]
                sen_dist = np.array(sensor) * raw_data[-1][i+1]
                new_sensor_pos = np.matmul(sen_dist,rot)
                prev_loc += 10
                new_sensor_pos[1]  = prev_loc
                data_points.append(new_sensor_pos)

            np_data_points = np.array(data_points)
            ax.scatter(np_data_points[:,0],np_data_points[:,1],np_data_points[:,2],c='red')
            plt.pause(.01)
            plt.draw()

            # maybe if the ball detects that its done, or erred, have it end here
            if msg == 'end':
                run = False;
                print("AN ERROR OCCURED")
            # user typed end, so stop the server 
            if user_input[0] == 'end':
                plt.show()
                user_input[0] = None
                run = False
                s = "end"
                serversocket.sendto(s.encode("utf8"), (arduino_ip , arduino_port));
                print("Halting ReconSphere...")
                time.sleep(2)


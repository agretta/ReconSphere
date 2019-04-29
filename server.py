#!/usr/bin/python3
import time 
import socket
import threading

print("Initializing ReconSphere Data Server")
# Create a UDP socket for this server
serversocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
port = 8347
serversocket.bind(('', port))

arduino_ip = "192.168.43.251"
arduino_port = 2390

data = []
init_gravity = []

user_input = [None]

# spawn a new thread to wait for input 
def get_user_input(user_input_ref):
    while user_input_ref[0] != 'end':
        user_input_ref[0] = input("Type 'end' to stop scan: ")


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
        msg = msg.decode("utf8");
        print(msg)
        decoded_msg = msg.split('|')
        init_gravity = decoded_msg
        print(init_gravity)

        # recieve distance data from the ReconSphere, and plot it
        while run:
            msg, addr = serversocket.recvfrom(1024)
            # decoded_msg = msg.decode("utf8");
            # msg = "111|222|333|1234|2345|3456|4567"
            msg = msg.decode("utf8");
            decoded_msg = msg.split('|')
            print(decoded_msg);
            print(msg);
            data.append([ decoded_msg[0:3] , decoded_msg[3] , decoded_msg[4] , decoded_msg[5] , decoded_msg[6] ]);

            #########################################
            # TODO: Do the graphing thing here, and fancy math
            #########################################

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


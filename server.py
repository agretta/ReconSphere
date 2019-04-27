#!/usr/bin/python3
import socket

# Create a UDP socket for this server
serversocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
port = 8888
serversocket.bind(('', port))

# Run the Server
print("Running Server")
while True:

    s = "Hello Arduino"
    serversocket.sendto(s.encode("utf8"), ("143.215.115.187" , 7));
    msg, addr = serversocket.recvfrom(1024)
    decoded_msg = msg.decode("utf8");
    decoded_msg = decoded_msg.split('|')
    print(decoded_msg);
    if msg == 'end':
        close = True
    print(decoded_msg)

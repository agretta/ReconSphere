#!/usr/bin/python3
import socket

# Create a UDP socket for this server
client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
port = 8889
client.bind(('', port))

while True:

    s = "Hello Server"
    client.sendto(s.encode("utf8"), ("143.215.115.187" ,8888));
    msg, addr = client.recvfrom(1024)
    decoded_msg = msg.decode("utf8");
    decoded_msg = decoded_msg.split('|')
    print(decoded_msg);
    if msg == 'end':
        close = True
    print(decoded_msg)

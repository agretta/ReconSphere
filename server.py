#!/usr/bin/python3
import socket

# Create a TCP socket for this server
serversocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
host = socket.gethostname()
port = 8888
serversocket.bind((host, port))
# serversocket.listen(5)

# Run the Server
while True:
    msg, addr = serversocket.recvfrom(1024)
    decoded_msg = msg.decode("utf8");
    decoded_msg = decoded_msg.split('|')
    print(decoded_msg);
    if msg == 'end':
        close = True
    print(msg)
    print(decoded_msg)

   # establish a connection with the client
   # clientsocket,addr = serversocket.accept()

   # print("Got a connection from %s" % str(addr))

   # msg = 'Thank you for connecting'+ "\r\n"
   # close = False;
   # while not close:
        # msg = clientsocket.recv(1024);
        # ball posistion + <x,y,z> of measure point

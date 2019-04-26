#!/usr/bin/python3
import socket

# Create a TCP socket for this server
serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = socket.gethostname()
port = 8888
serversocket.bind((host, port))
serversocket.listen(5)

# Run the Server
while True:
   # establish a connection with the client
   clientsocket,addr = serversocket.accept()

   print("Got a connection from %s" % str(addr))

   msg = 'Thank you for connecting'+ "\r\n"
   close = False;
   while not close:
        msg = clientsocket.recv(1024);
        # ball posistion + <x,y,z> of measure point
        decoded_msg = msg.decode("utf8");
        decoded_msg = decoded_msg.split('|')
        print(msg);
        if msg == 'end':
           close = True;
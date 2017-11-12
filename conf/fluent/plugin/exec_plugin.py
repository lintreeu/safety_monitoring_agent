#!/usr/bin/env python
# coding=UTF-8


import socket
import json

class Client(object):
    """
    A JSON socket client used to communicate with a JSON socket server. All the
    data is serialized in JSON. How to use it:
    
    """
    host = None
    port = None
    socket = None

    def __del__(self):
        self.close()

    def connect(self, host, port):
        self.socket = socket.socket()
        self.host = host
        self.port = port
        self.socket.connect((host, int(port)))
        return self

    def send(self, data):
        if not self.socket:
            raise Exception('You have to connect first before sending data')
        self._send(self.socket, data)
        return self

    def close(self):
        if self.socket:
            self.socket.close()
            self.socket = None

    def _send(self, socket, data):
        try:
            serialized = json.dumps(data)
        except (TypeError, ValueError):
            raise Exception('You can only send JSON-serializable data')
        


        try:
            # send the length of the serialized data first
            socket.send(('%d\n' % len(serialized)).encode())
            # send the serialized data
            socket.sendall(serialized.encode())
        

        #catch the exception of the disconnection
        except socket.error as e:

            # [Errno 104] connection reset by peer exception (aka ECONNRESET) on any call to send()
            if e.errno == errno.ECONNRESET:
                socket.connect(self.host, self.port)


if __name__ == '__main__':
    import time
    import random
    import sys

    host = 'localhost'
    port = '8087'
    client = Client()
    client.connect(host, port)
   
    #line : str
    for line in open(sys.argv[-1]):
        # transfer the string to dictionary type, because our monitoring agent can only deal with the dictionary
        line_dict = json.loads(line)
        client.send(line_dict)
    
    client.close()


    
    
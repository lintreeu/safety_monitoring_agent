import socket
import json

class Client(object):
    """
    A JSON socket client used to communicate with a JSON socket server. All the
    data is serialized in JSON. How to use it:
    
    """

    socket = None

    def __del__(self):
        self.close()

    def connect(self, host, port):
        self.socket = socket.socket()
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
        # send the length of the serialized data first
        socket.send(('%d\n' % len(serialized)).encode())
        # send the serialized data
        socket.sendall(serialized.encode())



if __name__ == '__main__':
    import time
    import random

    host = 'localhost'
    port = '8087'
    client = Client()
    client.connect(host, port)
    
    while True:

        metric_distance = random.uniform(0, 20)

        
        time.sleep(2)

        inputData = {
                    "tags": {"host": "alphabot","type": "distance", "device":"utrasonic_sensor"},
                    "fields" : {"front": round(metric_distance,3)}}
        print ('sending -> {}'.format(inputData))
     
        client.send(inputData)


    client.close()
#!/usr/bin/env python3

import json
import socket

class Server(object):
  """
    A socket server to receive the json type message
    
    Parameters
    ----------
    
    host : str, default is None
        Host address where the socket of the server binds
    port : str or int
    
    Attributes
    ----------  
    socket : object
        the return object from the socket.socket()
  """
  backlog = 5
  client = None

  def __init__(self, host, port):
    self.socket = socket.socket()
    self.socket.bind((host, int(port)))
    self.socket.listen(self.backlog)

  def __del__(self):
    self.close()

    #等待客戶端連線
  def accept(self):
    # if a client is already connected, disconnect it
    if self.client:
      self.client.close()
    self.client, self.client_addr = self.socket.accept()
    return self
  
     #發送資料
  def send(self, data):
    if not self.client:
      raise Exception('Cannot send data, no client is connected')
    self._send(self.client, data)
    return self

    #接收資料
  def recv(self):
    if not self.client:
      raise Exception('Cannot receive data, no client is connected')
    return self._recv(self.client)

    #關閉socket連線
  def close(self):
    if self.client:
      self.client.close()
      self.client = None
    if self.socket:
      self.socket.close()
      self.socket = None

      #將dict的資料轉成json tring byte 並送出其長度及資料
  def _send(self, socket, data):
    try:
      serialized = json.dumps(data)
    except (TypeError, ValueError):
      raise Exception('You can only send JSON-serializable data')
    # send the length of the serialized data first
    socket.send('%d\n' % len(serialized.encode()))
    # send the serialized data
    socket.sendall(serialized.encode())
   
    '''先接收string的長度後創建相應的記憶體空間
        每次接收1個string byte資料並存放在記憶體空間直到資料傳輸完成'''
  def _recv(self, socket):
    # read the length of the data
    length_str = ''
    char = socket.recv(1)
    
    #raise the exception if the client is diconnect
    if not char:
      raise ClientDisconnectionError()
   
    #接收字串長度值
    char = char.decode()
    while char != '\n':
      length_str += char
      char = socket.recv(1)
      char = char.decode()
    total = int(length_str)

    # use a memoryview to receive the data chunk by chunk 
    view = memoryview(bytearray(total)) #bytearray(total) create the bytearray that has enough space for the recieved message
    next_offset = 0
    while total - next_offset > 0: 
      recv_size = socket.recv_into(view[next_offset:], total - next_offset)
      next_offset += recv_size

    try:
      # the recieve data from the view.tobytes() is the byte string (the received data from the socket is the byte string)
      # because in python3, the string is the utf8 string or unicode string, we need to decode the byte string
      b_msg = view.tobytes() # b_msg: byte string
      msg = b_msg.decode() # decoding utf-8
      # transfer the jason string to the dictionary type. 
      deserialized = json.loads(msg)# deserialized: dict
    except (TypeError, ValueError):
      raise Exception('Data received was not in JSON format')
    return deserialized

class ClientDisconnectionError(Exception):
    def __init__(self, info = 'client disconncetion erro occurs'):
        self.info = info
    def __str__(self):
        return self.info


if __name__ == '__main__':
   
  host = 'localhost'
  port = '8087'

  # Server code:
  
  server = Server(host, port)
  print('the server is running...')
  print('waiting client connection...')
  server.accept()

  while True:
          
      try:
        data = server.recv()
      except ClientDisconnectionError:
        print('waiting client connection...')
        server.accept()

      # data now is: {'some_list': [123, 456]}
      print('type:{0}, data:{1}'.format(type(data),data))
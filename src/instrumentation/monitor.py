import json
import time

from fluent import sender

# monitor environment setup and run function
#setup: the name of the app(maybe use the uri)
class Monitor():
	
   

    ## monitor_tag depend on the fluentd's tag
    def __init__(self, fluentd_tag, monitor_tag = "monitor"): 
        self.logger = None
        self.fluentd_tag = fluentd_tag
        self.monitor_tag = monitor_tag        
        # type(self).port = zmq_port

    
    def setup(self):
        self.logger = sender.FluentSender(self.monitor_tag, host='localhost', port=24224, nanosecond_precision = True)

    def send(self, inputData):
       
        
        try:
            self.logger.emit_with_time(self.fluentd_tag, time.time(), inputData)

        except TypeError:
        	print("TypeError, data must be an dictionary")
    
if __name__ == '__main__':

    print('this is the test')
    mon = Monitor("1234")
    json_data = {"Name": "Anker", "Address" : "12244"} 
    mon.sendMessage(json_data, "123")


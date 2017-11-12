from osbrain import Agent

from safetyagent.rule import RuleManager, Rule, Environment, RuleSelector
from safetyagent.jsonsocketserver import Server, ClientDisconnectionError

class MonitoringAgent(Agent):
    """
    A base agent class which is to be served by an AgentProcess
    An AgentProcess runs a Pyro multiplexed server and serves one Agent
    object.

    Parameters
    ----------
             
    https://github.com/opensistemas-hub/osbrain/blob/master/osbrain/agent.py
    the parameters needed is same as the Agent class in the osbrain agent.py


    Attributes
    ----------
    _log: dict   for storing the json log file recieved from Fluentd

    Methods
    ----------

    """


    def __init__ (self, *args, **kwargs):

        super().__init__(*args, **kwargs)
        
        self._type = 'monitoring_agent'
        self.rule_manager = None
        self.server = None

    def type(self):
        return self._type

    def monitor(self, host, port):
        '''
        server: Server type
        host: string
        port: string or int
        '''
        self.server = Server(host, port)        


    def send(self, message_channel, msg):
        super().send(message_channel, msg)
       
    def set_rule(self, *rules):
        # check the type of the input parameters
        for rule in rules:

            if not isinstance(rule, Rule):
                raise Exception('TypeError: The type of parameters is not Rule')
            else:
                pass

        self.rule_manager = RuleManager(*rules)           

   
    def trigger_rule(self, message_channel = None):

        if self.server == None:
            raise Exception('Error: No monitor to listen. Shoud set up the monitor first')

        self.log_info('Waiting Fluentd connecting...')
        self.server.accept()        
      

        while True:
            try:
                msg = self.server.recv()

                             
                self.log_info(msg)               
                
                try:
                    self.rule_manager.trigger(msg)
                except AttributeError:
                    raise Exception('Error: Set the rule first')
                
                if message_channel is not None:
                    self.rule_manager.handle_verdict(self.send, message_channel)
                else:
                    self.rule_manager.handle_verdict(self.log_info)
                    

            except ClientDisconnectionError:
                self.log_info('Waiting clients connecting...')                
                self.server.accept()



class ControllingAgent(Agent):
    """
    A base agent class which is to be served by an AgentProcess
    An AgentProcess runs a Pyro multiplexed server and serves one Agent
    object.

        Parameters
        ----------
                 
        https://github.com/opensistemas-hub/osbrain/blob/master/osbrain/agent.py
        the parameters needed is same as the Agent class in the osbrain agent.py


        Attributes
        ----------
        _log: dict   for storing the json log file recieved from Fluentd

        Methods
        ----------

    """


    def __init__ (self, *args, **kwargs):

        super().__init__(*args, **kwargs)
       
        self._type = 'controlling_agent'
        self.controller = None

    def type(self):
        return self._type
  
      
    def set_controller(self, controller):
        self.controller = controller

    def update_controller(self, msg):
        self.controller.update(msg)
        self.log_info(self.controller.value)    
    def display_controller(self):
        self.controller.display()
    
    def control(self):
        self.controller.run()
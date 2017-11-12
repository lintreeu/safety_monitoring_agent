import time

from osbrain import SocketAddress
from osbrain import run_nameserver
from osbrain import run_agent
from osbrain import proxy 

import safetyagent.messagechannel as messagechannel
from safetyagent.agent import MonitoringAgent, ControllingAgent
from safetyagent.rule import Rule

# import the rules and controllers required by the agents
from safetyagent.controllerset import CloseDistanceController
from safetyagent.ruleset import tag_1, tag_2, ca_1, ca_2


Host = 'localhost'
Port = '8087'
# host, port for local and edge
local_socket_address = SocketAddress('127.0.0.1', 200)
edge_socket_address = SocketAddress('127.0.0.1', 210)



def controller_handeler(agent, message):
    agent.update_controller(message)
    agent.display_controller()
       


if __name__ == '__main__':

	# start the name server for multi-agent system 
	ns_local = run_nameserver(addr = local_socket_address)
	# run the two agents, monitoring agent (local) and controlling agent(local)
	agent_1 = run_agent('monitoring_agent_local', base = MonitoringAgent)
	agent_2 = run_agent('controlling_agent_local', base = ControllingAgent)


	# connect to the remote agent
	# ns_remote = proxy.NSProxy(nsaddr=edge_socket_address, timeout=10)
	# agent_3 = ns_remote.proxy('monitoring_agent_edge')
	agent_3 = run_agent('monitoring_agent_local_2', base = MonitoringAgent)


	'''
	set the Push-Pull communication pattern. The input of parameters are local monitoring_agent, 
	controlling agent, remote monitoring agent, channel_local, channel_remote,
	handler of controlling agent
	'''

	messagechannel.set(agent_1,agent_2,agent_3,'local_channel','edge_channel', controller_handeler)

	# set the port for the monitoring agent to listen the fluentd log message 
	agent_1.monitor(Host, Port)
	# set the rules need to be reacted by the monitoring agent
	rule1 = Rule(tag_1)
	rule1.add_condition_action(ca_1)
	agent_1.set_rule(rule1)

	# set the controller used by the controlling agent
	controller = CloseDistanceController()
	agent_2.set_controller(controller)   

	# running the setting 
	agent_1.trigger_rule('local_channel')
	agent_2.control()

import time

from osbrain import SocketAddress
from osbrain import run_nameserver
from osbrain import run_agent

from safetyagent.agent import MonitoringAgent



socket_address = SocketAddress('127.0.0.1', 210)
# System deployment

if __name__ == '__main__':

    ns = run_nameserver(addr = socket_address)

    agent_edge = run_agent('monitoring_agent_edge', base = MonitoringAgent)

    # agent_edge = ns.proxy('remote_agent')

    agent_edge.log_info('Monitoring agent is running')
    time.sleep(5)
    
    # while True:
    #     time.sleep(0.2)
    #     print('send the message: This is edge')
    #     agent_edge.send('edge_channel', 'This is edge')

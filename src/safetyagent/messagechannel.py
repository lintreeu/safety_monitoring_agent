# set the Push-Pull communication pattern 
'''set the pull-push communication pattern of the controlling agent(local), the remote 
        agent(monitoring agent on the edge server)'''




def set(monitoring_agent, controlling_agent, remote_agent, channel_local, channel_remote, handler):
'''
Parameters
----------
    monitoring_agent: MonitoringAgent  #本地端的monitoring agent
    controlling_agent: ControllingAgent #本地端的controlling agent
    remote_agent: MonitoringAgent #遠端的monitoring agent
    channel_local: string  #遠端agent與本地端controlling agent通訊頻道名稱
    channel_remote: string  #本地端agent與本地端controlling agent通訊頻道名稱
    handler: function  #callback function 決定當controlling agent接收資料後的function
'''

    addr_local = monitoring_agent.bind('PUSH', alias='local_channel')
    addr_edge =  remote_agent.bind('PUSH', alias='edge_channel')
    controlling_agent.connect(addr_edge, handler =handler)
    controlling_agent.connect(addr_local, handler =handler)
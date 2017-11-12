import time
import logging

class RuleSelector:
    '''
    select the corresponding environment which is able to be updated by the message

    Parameters
    -----------
    rule_list: Rule object

    '''
    def __init__(self, *environs):

        self.latest_msg = None
        self.environ_all = list(environs)
        self.selected_environ_list = []

    # return True if the message changes
    def _is_update(self, msg):

        if self.latest_msg is not msg:            
            return True
        else:
            return False

    #update the selected environment(not trigger the environment!!) list and return the selected environment
    def update(self, msg):

        if self._is_update(msg) is True:

            self.latest_msg = msg
            self.selected_environ_list = []
            for environ in self.environ_all:                          
                if environ.check(msg) is True:
                    self.selected_environ_list.append(environ)
            
            return self.selected_environ_list
       
        else:
            return self.selected_environ_list



class Environment:
    def __init__(self, log_mark):

        self.log_mark = log_mark
        self.tags = log_mark['tags'] # dict
        self.fields = log_mark['fields'] # list
        self.environ_fields = {} # dict
        self.rules_lists = [] # rules registor


        
    #msg : dict
    #check whether the enviroment can be updated by the  message 
    def check(self, msg):
        
        print(msg)
        # input('pause')
        if self.tags == msg['tags']: #compare tag dict

            field_dict = msg['fields']
            if set(self.fields) == set(key for key in field_dict.keys()): #compare fields list
                return True
        
        else:
            return False

    def update(self, msg):

        for key in self.fields:
            self.environ_fields[key] = msg['fields'][key]

    def add_rule(self, *rules):

        for rule in rules:
            if rule.condiction_action_lists is []: #no condiction_action in the rule
                raise Exception('No condiction_action in the rule input')   
            self.rules_lists.append(rule)



    def trigger(self):

        for rule in self.rules_lists:
            rule.trigger(self.environ_fields, self.fields) 
        

        
#log_marks : dict
class Rule:
    def __init__(self, *log_marks):
        
        
        for log in log_marks:
           
            if type(log) is not dict:
                raise Exception('the log mark is not the dictionary type')
            else:
                self.log_marks_list = list(log_marks)
        
        self.flag = False # when the rule is triggered, turn True
        self.condiction_action_lists = []
        self.environ_fields = []
        self.fields = []
        self.number = len(self.log_marks_list)
        self.verdict_list = []

        
        for i in range(self.number):
            
            # initialize evironment
            self.environ_fields.append('unknown')

            log_mark = self.log_marks_list[i]
            self.fields.append(log_mark['fields'])

    # get the log marks of the rule in the list
    def get_log_marks(self):
        return self.log_marks_list
    
    
        
        
    def trigger(self, environ_fields, fields):

        for i in range(self.number):

            if self.fields[i] == fields:
                             
                self.environ_fields[i] = environ_fields

        self._run()
        
    # return verdict list
    def get_verdict(self):
        return self.verdict_list

    def add_condition_action(self, *cas):

        for ca in cas:
            self.condiction_action_lists.append(ca) 

    def _rule(self):   
        
        logging.debug('trigger again...')
        
        self.flag = True
        logging.debug('triggering 1', self.flag)
        
        # clean the verdict triggered before
        self.verdict_list.clear()
        
        # run the condition-action rule. If the rule has the verdict, store it
        fields = self.environ_fields
        logging.debug('triggering 2',fields)        

        for ca in self.condiction_action_lists:          
                        
            verdict = ca(fields)
            logging.debug('triggering 3', verdict)

            if verdict is not None: 
                               
                self.verdict_list.append(verdict)
                logging.debug('triggering 4', self.verdict_list)
               
            else:
                ca(fields)
    
    def _run(self):

        if 'unknown' not in self.environ_fields:            
            self._rule()

        else:
            pass

   
class RuleManager:
    def __init__(self, *rules, env_cls = Environment, rule_selector_cls = RuleSelector):
      
'''
提供MonitoringAgent物件使用的接口。
具有當新的message接收後會觸發Environment物件方法，以及處理當Rule物件產生的verdict方法
'''  
        
        self.rule_selector_cls = RuleSelector
        self.env_cls = Environment        
        self.environ_list = []      
        self.log_marks_list = []
        self.rules_list = list(rules)

        self.selector = None
        # add all the log marks of the rule and exclude the same log mark 
        for rule in rules:
            for log_mark in rule.log_marks_list:

                if log_mark not in self.log_marks_list:
                    self.log_marks_list.append(log_mark)

        print('The log marks list of the rule manager:')
        print(self.log_marks_list)
        self._set_environ()
        self._set_rule_selector()

    def _set_environ(self):

        for log_mark in self.log_marks_list:
            
            # add the environment to the envron_list list
            environ = self.env_cls(log_mark)
            self.environ_list.append(environ)

            # add the rules to the environment
            for rule in self.rules_list:
                if log_mark in rule.log_marks_list:
                    environ.add_rule(rule)


    
    def _set_rule_selector(self):

        environ_list = tuple(self.environ_list)
        self.selector = self.rule_selector_cls(*environ_list)

    def trigger(self, msg):
        
        selected_environ = self.selector.update(msg) #list 
        for environ in selected_environ:          
            
            environ.update(msg)
            environ.trigger()
            
    def handle_verdict(self, handler, message_channel = None):
        
        for rule in self.rules_list:
           
            if rule.flag is True and rule.verdict_list is not []:
                    rule.flag = False
                    for verdict in rule.verdict_list:
                        
                        if message_channel is None:
                            handler(verdict)
                        else:
                            handler(message_channel, verdict)
            else:
                pass    

       


if __name__ == '__main__':
    '''testing block'''
    import logging
    import time
    import random
    
    logging.basicConfig(level = logging.INFO)

    '''log marks'''
    dictA =  {
            "tags":{"host": "alphabot","type": "distance", "device":"utrasonic_sensor"},
            "fields": ["front","front_2"]
            }
    dictB = {
            "tags": {"host": "alphabot","type": "position", "device":"infrared_light"},
            "fields" : ["center_ratio","center_ratio_2"]
            }
    messageA =  {
                "tags":{"host": "alphabot","type": "distance", "device":"utrasonic_sensor"},
                "fields": {"front": 3, "front_2": 4}
                }
    messageB =  {
                "tags": {"host": "alphabot","type": "position", "device":"infrared_light"},
                "fields" : {"center_ratio": 44, "center_ratio_2":55}    
                 }
    '''condition_action '''
    def ca_1(fields):
        if fields[0]['front'] + fields[1]['center_ratio'] > 10:
       
            return 'run'
        else:
            return 'stop'
    # def ca_2(fields):
    #     if fields[0]['front'] - fields[1]['center_ratio'] < 6:
    #         return 'gogo'
    #     # print(fields[0]['front'] - fields[0]['front_2'])
    #     else:
    #         return 'nono'
    

    '''
        the parameters for the Rule should be given in order
        In this example, we see that fields[0] is based dicA, so dicA is fronter than dictB(fields[1]) 
    '''

    rule1 = Rule(dictA, dictB)
    rule1.add_condition_action(ca_1)
    manager = RuleManager(rule1)
    # environA = Environment(dictA)
    # environA.add_rule(rule1)
    # environB = Environment(dictB)
    # environB.add_rule(rule1)
    # ruleselector = RuleSelector(environA, environB)

    def send_msg(msg):
        print(msg)

    while True:

        time.sleep(1)
        
       
        
        messageA['fields']['front'] = random.randint(0,10)
        messageB['fields']['center_ratio'] = random.randint(0,10)
        messageA['fields']['front_2'] = random.randint(0,10)
        messageB['fields']['center_ratio_2'] = random.randint(0,10)
        msg_c = messageA['fields']['front'] + messageB['fields']['center_ratio']
        for msg in (messageA, messageB):
            print('\n',msg_c)
            # print(msg['fields']['front'])
            # input('pause')
            manager.trigger(msg)
            manager.handle_verdict(send_msg)
   



'''
comment
----------
aka can write many if_else
aca(fields) can change any name
'''
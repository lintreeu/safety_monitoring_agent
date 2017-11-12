tag_1 =  {
        "tags":{"host": "alphabot","type": "distance", "device":"utrasonic_sensor"},
        "fields": ["front"]
        }
tag_2 = {
        "tags": {"host": "alphabot","type": "position", "device":"infrared_light"},
        "fields" : ["center_ratio"]
        }

def ca_1(fields):
        if fields[0]['front'] < 30:
            return 'emergency_control'
        else:
            return 'normal_control' 
        
def ca_2(fields):
    print(fields[0]['front'] - fields[1]['center_ratio']) 
    print(fields[0]['front'] - fields[0]['front_2'])
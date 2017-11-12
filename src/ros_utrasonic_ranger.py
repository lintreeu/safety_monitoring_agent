#!/usr/bin/python
# -*- coding:utf-8 -*-

import time
import math
import RPi.GPIO as GPIO
   

class Measurement(object):
    '''Create a measurement using a HC-SR04 Ultrasonic Sensor connected to 
    the GPIO pins of a Raspberry Pi.
    Metric values are used by default. For imperial values use
    unit='imperial'
    temperature=<Desired temperature in Fahrenheit>
    '''

    def __init__(self,
                 trig_pin,
                 echo_pin,
                 temperature=28,
                 unit='metric',
                 round_to=1
                 ):
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self.temperature = temperature
        self.unit = unit
        self.round_to = round_to
        self.sonar_signal_off = 0
        self.sonar_signal_on = 0

    def raw_distance(self, sample_size=11, sample_wait=0.1):
        '''Return an error corrected unrounded distance, in cm, of an object 
        adjusted for temperature in Celcius.  The distance calculated
        is the median value of a sample of `sample_size` readings.
        
        '''

        if self.unit == 'imperial':
            self.temperature = (self.temperature - 32) * 0.5556
        elif self.unit == 'metric':
            pass
        else:
            raise ValueError(
                'Wrong Unit Type. Unit Must be imperial or metric')

        speed_of_sound = 331.3 * math.sqrt(1+(self.temperature / 273.15))
        sample = []
        # setup input/output pins
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trig_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        
        for distance_reading in range(sample_size):
            GPIO.output(self.trig_pin, GPIO.LOW)
            time.sleep(sample_wait)
            GPIO.output(self.trig_pin, True)
            time.sleep(0.00001)
            GPIO.output(self.trig_pin, False)
            echo_status_counter = 1
            time_passed = 0
            while GPIO.input(self.echo_pin) == 0:
                if echo_status_counter < 100000:

                    self.sonar_signal_off = time.time()
                    echo_status_counter += 1
                else:
                    print(echo_status_counter)
                    raise SystemError('Echo pulse was not received',echo_status_counter)
            while GPIO.input(self.echo_pin) == 1:
                self.sonar_signal_on = time.time()
            time_passed = self.sonar_signal_on - self.sonar_signal_off
            distance_cm = time_passed * ((speed_of_sound * 100) / 2)
            sample.append(distance_cm)
        sorted_sample = sorted(sample)
        # Only cleanup the pins used to prevent clobbering
        # any others in use by the program
        GPIO.cleanup((self.trig_pin, self.echo_pin))
        return sorted_sample[sample_size // 2]

    def depth_metric(self, median_reading, hole_depth):
        '''Calculate the rounded metric depth of a liquid. hole_depth is the
        distance, in cm's, from the sensor to the bottom of the hole.'''
        return round(hole_depth - median_reading, self.round_to)

    def depth_imperial(self, median_reading, hole_depth):
        '''Calculate the rounded imperial depth of a liquid. hole_depth is the
        distance, in inches, from the sensor to the bottom of the hole.'''
        return round(hole_depth - (median_reading * 0.394), self.round_to)

    def distance_metric(self, median_reading):
        '''Calculate the rounded metric distance, in cm's, from the sensor
        to an object'''
        return round(median_reading, self.round_to)

    def distance_imperial(self, median_reading):
        '''Calculate the rounded imperial distance, in inches, from the sensor
        to an oject.'''
        return round(median_reading * 0.394, self.round_to)


if __name__ == "__main__":
        
    from instrumentation.monitor import Monitor
    import rospy
    from std_msgs.msg import Float64
    
    monitor = Monitor("data_utrasonic")
    monitor.setup() 

    '''Calculate the distance of an object in centimeters using a HCSR04 sensor
        and a Raspberry Pi. This script allows for a quicker reading by
        decreasing the number of samples and forcing the readings to be
        taken at quicker intervals.'''

    trig_pin = 22
    echo_pin = 27
    flag = True

    #  Create a distance reading with the hcsr04 sensor module
    value = Measurement(trig_pin, echo_pin)     
    
    #initialize the ros message channel
    rospy.init_node('UltrasonicRanger', anonymous=True)  
    pub = rospy.Publisher('front_distance', Float64, queue_size=10) 

    try:
        while True:
            raw_measurement = value.raw_distance(sample_size=5, sample_wait=0.01)
               # Calculate the distance in centimeters

            metric_distance = value.distance_metric(raw_measurement)
                      
                
             #The log data 
            inputData = {
                        "tags": {"host": "alphabot","type": "distance", "device":"utrasonic_sensor"},
                        "time": round(time.time(),3),
                        "fields" : {"value": metric_distance}}              
            #send the log data to fluentd
            monitor.send(inputData)

            #send the control message(here is metric_distance) through the ROS master to the controlling agent
            pub.publish(metric_distance)

            print("The Distance = {} centimeters".format(metric_distance))

    except KeyboardInterrupt:
            print("Measurement stopped by User")
            GPIO.cleanup()
#!/usr/bin/python
# -*- coding:utf-8 -*-
import RPi.GPIO as GPIO
import time
import math
#import AlphaBot
PI = 3.14


from instrumentation.monitor import Monitor

monitor = Monitor("data_position")
monitor.setup()


class PositionEstimator: 
  global PI

  def __init__(self, motor, wheel_encoder_left, wheel_encoder_right, wheel_base_length):
    self.wheel_base_length = wheel_base_length
    self.motor = motor
    
    self.t0 = 0
    self.t1 = 0
    self.position = [0, 0, 0]
    self.twist = [0, 0 ,0]
    

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(self.encoder_left.inputPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(self.encoder_right.inputPin,GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(self.encoder_left.inputPin, GPIO.FALLING, callback=self._update_postion_left)
    GPIO.add_event_detect(self.encoder_right.inputPin, GPIO.FALLING, callback=self._update_position_right)
  

  def _update_postion_left(self, channel):
    
    self.update_encoder_state(self.motor)
    self.encoder_left.triggierTicks()
    self._estimate_movement()
    self.encoder_left.updateTicks()
    self.displayPosition()

  def _update_position_right(self, channel):
    
    self.update_encoder_state(self.motor)
    self.encoder_right.triggierTicks()
    self._estimate_movement()
    self.encoder_right.updateTicks()
    self.displayPosition()

  #calculate the wheel movement after the duration time delta_t 
  def _estimate_movement(self):

    #calculate the time duration between between the current odemetry input 
    #and the last odemetry input
    
    self.t1 = time.time()
    delta_t = self.t1 - self.t0
    self.t0 = time.time()
    
    d_ticks_left = self.encoder_left.getTicks() - 0 
    d_ticks_right = self.encoder_right.getTicks() - 0
    d_left_wheel = 2*PI*self.encoder_left.radius*( d_ticks_left / self.encoder_left.ticksPerTurn)
    d_right_wheel = 2*PI*self.encoder_right.radius*( d_ticks_right / self.encoder_right.ticksPerTurn )
    d_center = 0.5 * ( d_left_wheel + d_right_wheel )
   
    
    prev_x, prev_y, prev_theta = self.position
    new_x = prev_x + ( d_center * math.cos( prev_theta ) )
    new_y = prev_y + ( d_center * math.sin( prev_theta ) )
    new_theta = prev_theta + ( ( d_right_wheel - d_left_wheel ) / self.wheel_base_length )
    new_theta = new_theta%(2*PI)
    position = (new_x, new_y, new_theta)
    twist = [(new_x - prev_x)/delta_t, (new_y - prev_y)/delta_t, (new_theta - prev_theta)/delta_t]
    
    self.position = position
    self.twist = twist

    inputData = {
                "tags": {"host": "alphabot","type": "position", "device":"position_estimater"},
                "time": round(time.time(),3),
                "fields" : {"x" : round(new_x,2),
                            "y" : round(new_y,2),
                            "theta" : round((new_theta/PI)*180,2),
                            "x_velocity" : round(twist[0],2),
                            "y_velocity" : round(twist[1],2),
                            "theta_velocity" : round((twist[2]/PI)*180,2)}                }



    monitor.send(inputData)
    self.displayPosition()




  def displayPosition(self):
    
    global PI
    x, y, theta_rad = self.position
    x, y, theta_degree = x, y, (theta_rad/PI)*180
    v_x, v_y, v_theta = self.twist
    v_x, v_y, v_theta_dgree = v_x, v_y, (v_theta/PI)*180
    print("x: {0}, y: {1}, theta: {2}".format(x, y, theta_degree))  
    print("v_x: {0}, v_y: {1}, v_theta: {2}".format(v_x, v_y, v_theta_dgree))


class WheelEncoder:

  'Encapsulates the attributes and methods to use a wheel encoder sensor'

  def __init__(self, inputPin, ticksPerTurn, radius):
    self.inputPin = inputPin
    self.ticksPerTurn = float(ticksPerTurn)
    self.radius = radius
    
    self.ticks = 0
    self.accticks = 0      
    self.state = "Unknown"   
  
  def setState(self, cmd):
    self.state = cmd

  def triggierTicks(self):
    if self.state == "add":
      self.ticks += 1
      self.accticks += 1
      print(self.ticks)
    if self.state == "abst":
      self.ticks -= 1
      self.accticks -= 1
      print(self.ticks)
  


  def getTicks(self):
    return self.ticks

  def getAccTicks(self):
    return self.accticks

  def updateTicks(self):
    self.ticks = 0

  def display(self):
    print(self.accticks, self.ticks)

## test part
class Motor:
  def __init__(self):
    pass
    self.state = "forward"
  def getState(self):
    return self.state

if __name__ == '__main__':

  CE1 = 7 #CNTL the right side of the wheel photo interupter (because the car runs reversely)
  CE0 = 8 #CNTR the left side ....
  ticksPerTurn  = 20
  radius = 3.35
  wheel_base_length = 14
  
  

  motor = Motor()
  encoder_right = WheelEncoder(CE1, ticksPerTurn , radius)
  encoder_left = WheelEncoder(CE0, ticksPerTurn, radius)
  estimator = PositionEstimator(motor, encoder_left, encoder_right, wheel_base_length)
 
  while True:
    pass
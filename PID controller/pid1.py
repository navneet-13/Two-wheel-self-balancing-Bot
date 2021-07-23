#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class PID_Controller:
    '''
    General PID control class. 
    '''

    def __init__(self, Kp, Ki, Kd):
        '''
        Constructs a new PID_Controller object.
        '''
        
        # Parameters
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        # State variables
        self.Eprev = 0
        self.Stdt = 0
        self.t = 0
    def tune(self,KpNew,KiNew,KdNew):
        self.Kp = KpNew
        self.Ki = KiNew
        self.Kd = KdNew
    def getCorrection(self, target, actual, dt=.8):
        '''
        Returns current PID correction based on target value and actual value.
        '''
              
        E = target - actual
    
        # dE / dt
        dEdt = (E - self.Eprev) / dt if self.t > 0 else 0
        
        #if abs(dEdt) > 1: # XXX Why?
        #    dEdt = 0 # XXX Why?
        
        # Integral E / dt XXX not how you actually integrate
        self.Stdt += E*dt if self.t > 0 else 0# (E + self.Eprev)*dt if self.t > 0 else 0
   
        # Correcting

        correction = self.Kp*E + self.Ki*self.Stdt + self.Kd*dEdt
    
        # Update
        self.t += 1
        self.Eprev = E

        return correction
        
        

   

Kp =10.5
Ki =1.4
Kd =.55


class SelfBalance:
    def __init__(self):
        self.pub = rospy.Publisher("cmd_vel",Twist,queue_size =1)
        self.subscriber = rospy.Subscriber("state",Float64,self.callback)
        self.Subscriber = rospy.Subscriber("setpoint",Float64, self.setpoint)     

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.pubx = PID_Controller(self.Kp,self.Ki,self.Kd)
    def setpoint(self,msg):
    	self.setPoint = msg.data
    def callback(self,msg):
     
        y = msg.data
        
        vel = Twist()
        
        if abs(y)>1: 
        	xvel = 0
        else:
        	xvel = self.pubx.getCorrection(self.setPoint,y)
        
        
        print(xvel)
        
        vel.linear.x = -1*xvel/2
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x =0
        vel.angular.y = 0
        vel.angular.z = 0

        self.pub.publish(vel)


def main():
    '''Initializes and cleanup ros node'''
    rospy.init_node('SelfBalance', anonymous=True)
    ic = SelfBalance()
    
    rospy.spin()
if __name__ == '__main__':
    main()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    

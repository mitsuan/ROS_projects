#!/usr/bin/env python

import cv2
import numpy
import rospy
from math import * 
from robosoccer.msg import error_param


pub = rospy.Publisher('robot_motion', String,queue_size=10)

#PID function for angle control
prev_theta = float(0.0)
w = float(0.0)
p = float(0.0)
i = float(0.0)
d = float(0.0)
def pid_W(theta):
    global prev_theta,w,p,i,d
    kp = cv2.getTrackbarPos('KP','PID_W')/10
    ki = cv2.getTrackbarPos('KI','PID_W')/10
    kd = cv2.getTrackbarPos('KD','PID_W')/10
    p = theta
    d = theta - prev_theta
    i = i + theta
    if(i>200):
        i = 200
    if(i<-200):
        i = -200
    w = kp * p + ki * i + kd * d
    if(w > 300):
        w = 300
    if(w < -300):
        w = -300
    pp = "{:.3f}".format(p)
    ii = "{:.3f}".format(i)
    dd = "{:.3f}".format(d)
    ww = "{:.3f}".format(w)
    prev_theta = theta
    #print(pp,' , ',ii,' , ',dd,' , ',ww)
    return int(float(ww))


#PID function for velocity control
prev_dist = float(0.0)
v = float(0.0)
_p = float(0.0)
_i = float(0.0)
_d = float(0.0)
def pid_D(dist):
    global v,_p,_i,_d,prev_dist
    kp = cv2.getTrackbarPos('KP','PID_D')/10
    ki = cv2.getTrackbarPos('KI','PID_D')/10
    kd = cv2.getTrackbarPos('KD','PID_D')/10
    _p = dist
    _d = dist - prev_dist
    _i = _i + dist
    if(_i > 200):
        _i = 200
    if(_i < -200):
        _i = -200
    v = _p * kp + _i * ki + _d * kd
    if( v > 150):
        v = 150
    pp = "{:.3f}".format(_p)
    ii = "{:.3f}".format(_i)
    dd = "{:.3f}".format(_d)
    vv = "{:.3f}".format(v)
    #print(pp,' , ',ii,' , ',dd,' , ',vv)
    prev_dist = dist
    return int(float(vv))


def callback(param):

    rospy.loginfo("angle is : %f and distance is: %f" % (param.angle,param.distance))

	v_m = float(0.0);
	if(dist > 50):
		v_m=pid_D(param.distance);

	theta_2=param.angle + (pi/2)
	x=v_m*cos(theta_2)
	y=v_m*sin(theta_2)
		
	w=pid_W(degrees(param.angle));
	

	velocity_msg=str(x)+' '+str(y)+' '+str(w);
	
	#publishing robot's motion values :x,y,w
	pub.publish(velocity);



def listener():
    rospy.init_node('error_listener', anonymous=True)
    rospy.Subscriber("error_data", error_param, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

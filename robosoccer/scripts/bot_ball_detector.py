#!/usr/bin/env python
# license removed for brevity
import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from robosoccer.msg import error_param


def track(hsv,lower,upper):

      global img
      
      mask = cv2.inRange(hsv, lower, upper)      

      erodelement = np.ones((5,5),np.uint8)
      dilatelement = np.ones((8,8),np.uint8)

      erosion = cv2.erode(mask,erodelement,iterations = 1)
      dilation = cv2.dilate(mask,dilatelement,iterations = 1)

      dilation = cv2.dilate(mask,dilatelement,iterations = 1)
      erosion = cv2.erode(mask,erodelement,iterations = 1)

      contour, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

      t = int(0)
      max_area = int(0)
      cnt = int(0)
      x = int(0)
      y = int(0)
      
      for cnt in range(len(contour)):
            area = cv2.contourArea(contour[cnt])
            if(area>max_area):
                max_area = area
                t = cnt

      if(len(contour)>0):
          (x,y),radius = cv2.minEnclosingCircle(contour[t])
          center = (int(x),int(y))
          radius = int(radius)
          cv2.drawContours(img,contour[t],0,(0,0,255),-1)   # draw contours in green color
          cv2.circle(img,center,radius,(0,0,0),2)

      return [x,y]

	
def detect_bot_front(hsv):
	#for color Green      
	g_lower = np.array([73, 152, 129])
	g_upper = np.array([83, 255, 255])

	g_cX = 0
	g_cY = 0            
	
	[g_cX,g_cY] = track(hsv,g_lower,g_upper)
	
	return [g_cX,g_cY]

def detect_bot_back(hsv):
	#for color Maroon
	m_lower = np.array([92, 85, 74])
	m_upper = np.array([195, 255, 166])

	m_cX = 0
	m_cY = 0

	[m_cX,m_cY] = track(hsv,m_lower,m_upper)

	return [m_cX,m_cY]


def detect_bot_centroid(hsv):
	
	front=detect_bot_front(hsv);
	back =detect_bot_back( hsv);

	#center of line
	bot_x= (front[0] + back[0])/2
    bot_y = (front[1] + back[1])/2

	return [bot_x,bot_y]
	


def detect_ball_centroid(hsv):
	#for color Orange
	o_lower = np.array([160, 83, 0])
	o_upper = np.array([188, 186, 255])

	ball_X = 0
	ball_Y = 0

	[ball_X,ball_Y] = track(hsv,o_lower,o_upper)
	
	return [ball_X,ball_Y];

def distance_ball_bot(bot_centroid,ball_centroid):
	return	((ball_centroid[0]-bot_centroid[0])**2 + (ball_centroid[1]-bot_centroid[1])**2)**0.5

def angle_ball_bot(hsv,bot_centroid,ball_centroid);

	#finding angle in radians     
	theta_1 = atan2(detect_bot_front(hsv)[1]-bot_centroid[1] , detect_bot_front(hsv)[0]-bot_centroid[0]) - atan2(ball_centroid[1]-bot_centroid[1] ,ball_centroid[0]-bot_centroid[0])

	if (theta_1 < -pi):
		theta_1 += pi*2
	if (theta_1 > pi):
		theta_1 -= pi*2
	
	return theta_1;
	


def detector():
    pub = rospy.Publisher('error_data', error_param, queue_size=10)
    rospy.init_node('detector', anonymous=True)
    rate = rospy.Rate(10) # 10hz
	cam = cv2.VideoCapture(1)

    while not rospy.is_shutdown():
		
		ret,img = cam.read()

		hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

		bot_centroid=detect_bot_centroid(hsv); 
		ball_centroid=detect_ball_centroid(hsv);

		ball_bot_dist=dist_ball_bot(bot_centroid,ball_centroid)

		ball_bot_angle=angle_ball_bot(hsv,bot_centroid,ball_centroid);

		error_msg=error_param();
		error_msg.angle=ball_bot_angle;
		error_msg.distance=ball_bot_dist;

        rospy.loginfo(error_msg)

        pub.publish(error_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        detector()

    except rospy.ROSInterruptException:
        pass

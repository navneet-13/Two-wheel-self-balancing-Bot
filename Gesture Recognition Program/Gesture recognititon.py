#!/usr/bin/env python
import cv2
import mediapipe as mp
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import time

mpHands = mp.solutions.hands
hands = mpHands.Hands()
mpDraw = mp.solutions.drawing_utils


def rec():
	
	pub_1 = rospy.Publisher("cmd_vel",Twist,queue_size =1)
	pub_2 = rospy.Publisher("setpoint",Float64,queue_size =1)
	
	vel_1 = Twist()
	vel_1.linear.x = 0
	vel_1.linear.y = 0
	vel_1.linear.z = 0
	vel_1.angular.x =0
	vel_1.angular.y = 0
	vel_1.angular.z = 1.0
	
	vel_2 = Twist()
	vel_2.linear.x = 0
	vel_2.linear.y = 0
	vel_2.linear.z = 0
	vel_2.angular.x =0
	vel_2.angular.y = 0
	vel_2.angular.z = -1.0
	cap = cv2.VideoCapture(0)
	while(True):
		ret, vid = cap.read()
		frame = cv2.flip(vid , 1)
		#frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		rgb = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
		results = hands.process(rgb)
		#print(results.multi_hand_landmarks)



		if results.multi_hand_landmarks:
			for handLms in results.multi_hand_landmarks:
				a, b, c = frame.shape
				x = []
				y = []
				for id, lm in enumerate(handLms.landmark):

					for i in range(21):
						if id == i:
							x.append(lm.x*b)
							y.append(lm.y*a)
				if y[0]>y[3] and y[4] > y[8] and y[4] > y[12] and y[4] > y[16] and y[4] > y[20] and x[4]+20 < x[8] and x[8] < x[12]and x[12] < x[16] and x[16] < x[20] and x[8] < x[10] 					   and x[16] < x[18]:
					print("start")
					pub_2.publish(.02)
				elif y[0]>y[3] and y[4] > y[8] and y[4] > y[12] and y[4] > y[16] and y[4] > y[20] and x[4] > x[8]+20 and x[8] > x[12]and x[12] > x[16] and x[16] > x[20] and   					x[8] > x[10] and x[16] > x[18]:
					print("start")
					pub_2.publish(.02)
				elif x[4]>x[8] and x[0]>x[3] and y[8]<y[10] and x[6]<x[10] and x[6]<x[14] and x[6]<x[18] and y[3]<y[8] :
					print('right')
					pub_1.publish(vel_1)
					time.sleep(.055)
				elif x[4] < x[8] and  x[0]<x[3] and y[8]<y[10] and x[6] > x[10] and x[6] > x[14] and x[6] > x[18] and y[3]  < y[8]:
					print('left')
					pub_1.publish(vel_2)
					time.sleep(.055)
				elif y[0]>y[1] and y[0]>y[4] and y[0]>y[8] and y[0]>y[12] and y[0]>y[16] and y[6]<y[8] and y[10]<y[12] and y[14]<y[16] and y[18]<y[20] and y[4]<y[6]:
					print("stop")
					pub_2.publish(-.02)
					
				else:
					print("No")
					pub_2.publish(0.00)	


				mpDraw.draw_landmarks(frame, handLms, mpHands.HAND_CONNECTIONS)


		cv2.imshow("frame", frame)
		if cv2.waitKey(1) & 0xFF == ord("q"):
			break
		
	cap.release()
	
def main():
	rospy.init_node("Command" , anonymous = True)
	rec()
	rospy.spin()
if __name__ == '__main__':
    main()


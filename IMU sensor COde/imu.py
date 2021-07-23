#!/usr/bin/env python
import rospy
import tf
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64

def callback(x, y, z):
	print("Orientation:")
	print("     roll :" , x)
	print("     pitch:" , y)
	print("     yaw  :" , z)
	
def conversion(msg):
	quaternion = (
		msg.orientation.x,
		msg.orientation.y,
		msg.orientation.z,
		msg.orientation.w,
		)
		
	euler = tf.transformations.euler_from_quaternion(quaternion)
	
	roll  = euler[0]
	pitch = euler[1]
	yaw   = euler[2]  
	print("     pitch:" , pitch)
	
	pub = rospy.Publisher('state', Float64, queue_size=10)
	
	while not rospy.is_shutdown():
		pub.publish(pitch)
		break;

#	callback(roll, pitch, yaw)
	
	
def extract():
	
	rospy.init_node('extract', anonymous = True)
	
	rospy.Subscriber("imu", Imu, conversion)
	
	rospy.spin()
	
if __name__ == '__main__':
	extract()

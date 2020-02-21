#!/usr/bin/python
import rospy
import numpy as np
from std_msgs.msg import Int32 
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import tf
import time

class RosbotFollower:
	def __init__(self):
		# set up publisher
		self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
		self.key_pressed = rospy.Publisher('/rgb_classification/keypressed', Int32, queue_size = 1)
		# set up the subscriber
		self.pose_subscriber = rospy.Subscriber('/pose', PoseStamped, self.retrive_pose)
		# 1: Orange 2: Basketball 3: CardboardBox 4: WoodenBox 5: Computer 6: Book 7: Watermelon 8:Apple
		# 9: OrangeSphere 10: BrownBox 11: BlackBox 12: GreenSphere 13: Box 14: Sphere 15: Object
		############################
		#1:0:"Object",1:"Box", 2:"Sphere", 3:"BrownBox", 4:"BlackBox", 5:"OrangeSphere", 6:"GreenSphere", 7:"CardboardBox", 8:"WoodenBox", 
        #9:"Computer", 10:"Book", 11:"Orange", 12:"Basketball", 13:"Watermelon",14:"Apple"}
		
		self.gotopt = 0
		self.init_pose = np.zeros((3,1))
		self.on_off = 0
		self.on_off_final_class = 0
		#self.prev_pose = self.robot_pose

		# wait one second to give subscriber time to connect
		rospy.sleep(1)

	def retrive_pose(self, data):

		#self.robot_pose = data
		robot_pose = np.zeros((3,1))
		robot_pose[0] = data.pose.position.x
		robot_pose[1] = data.pose.position.y
		orien_x = data.pose.orientation.x
		orien_y = data.pose.orientation.y
		orien_z = data.pose.orientation.z
		orien_w = data.pose.orientation.w
		anglesFromOrien = tf.transformations.euler_from_quaternion([orien_x, orien_y, orien_z, orien_w])
		robot_pose[2] = anglesFromOrien[2]

		if self.on_off == 0:
			self.init_pose = robot_pose;
			self.on_off = 1
		e = 0.4
		error_angle = 0.05
		kp = 2
		maxV = 0.09
		wheel2Center = 23.5/200
		closeEnough = 0.08
		waypoints = np.array([[2.4, 0.0, 0.0], 
			                  [2.4, 1, np.pi/2]])

		#1:0:"Object",1:"Box", 2:"Sphere", 3:"BrownBox", 4:"BlackBox", 5:"OrangeSphere", 6:"GreenSphere", 7:"CardboardBox", 8:"WoodenBox", 
        #9:"Computer", 10:"Book", 11:"Orange", 12:"Basketball", 13:"Watermelon",14:"Apple"}
		#The following three sets of parameters should be carefully tuned
		init_feature = np.array([0,0])
		init_feature_level = np.array([0,0])
		measurements_to_make = np.array([2,2])
		desiredV, desiredW, distance, current_theta = self.visitWaypoints(self.gotopt, waypoints, e, robot_pose)
		cmdV, cmdW = self.limitCmds(desiredV, desiredW, maxV, wheel2Center)
		n = len(waypoints) - 1
		vel_msg = Twist()
		desiredPose = waypoints[self.gotopt]
		orient = self.toWholeAngle(np.arctan2(desiredPose[1] - robot_pose[1], desiredPose[0] - robot_pose[0]))
		desiredAngle = waypoints[self.gotopt][2]
		desiredAngle = self.toWholeAngle(desiredAngle)

		if distance <= closeEnough and abs(current_theta - desiredAngle) < error_angle and self.gotopt == n:
			#all waypoints are clear
			if self.on_off_final_class == 0:
				target_feature_level = init_feature_level[self.gotopt]
				target_feature = init_feature[self.gotopt]
				num_measure = measurements_to_make[self.gotopt]
				self.key_pressed.publish(num_measure.astype(np.int32)*19 + target_feature.astype(np.int32))
				self.on_off_final_class = 1
			print('All waypoints are done')
			vel_msg.linear.x = 0.0
			vel_msg.angular.z = 0.0
		else:
			if distance > (closeEnough + 0.25) and abs(self.toWholeAngle(current_theta) - orient) > 0.18 and abs(self.toWholeAngle(current_theta) - orient) < 6.10:
				#In this branch the robot rotates so that it points the waypoinb to go
				#print('Diff orient',abs(self.toWholeAngle(current_theta) - orient))
				cmdW = self.angleControl(orient,current_theta)
				#print('cmdW in orient',cmdW)
				cmdV = 0.0
				vel_msg.linear.x = cmdV
				vel_msg.angular.z = cmdW
			else:
				if distance <= closeEnough and self.gotopt <= n:
					#rotate to the specified orientation
					current_theta = self.toWholeAngle(current_theta)
					if abs(current_theta - desiredAngle) > error_angle:
						cmdW = self.angleControl(desiredAngle,current_theta)
						cmdV = 0.0
						#print('cmdW',cmdW)
						#print('Angle Diff',abs(current_theta - desiredAngle))
						vel_msg.linear.x = cmdV
						vel_msg.angular.z = cmdW
					else:
						#begin test decision
						target_feature_level = init_feature_level[self.gotopt]
						target_feature = init_feature[self.gotopt]
						num_measure = measurements_to_make[self.gotopt]
						self.key_pressed.publish(num_measure.astype(np.int32)*19 + target_feature.astype(np.int32))
						time.sleep(10)
						print("Let's continue ")
						self.gotopt = self.gotopt + 1
				else:
					#go forward
					vel_msg.linear.x = 1.2*cmdV
					vel_msg.angular.z = 0.0		

		self.velocity_publisher.publish(vel_msg)
			
	def angleControl(self, desiredAngle, cur_theta):
		cmdW = (desiredAngle - cur_theta)*0.03
		if abs(desiredAngle - cur_theta) > np.pi:
			cmdW = -cmdW
		return cmdW


	def visitWaypoints(self, gotopt, waypoints, e, robot_pose):
		# unpack the data
		# self.prev_pose = self.robot_pose
		current_x = robot_pose[0]
		current_y = robot_pose[1]
		current_theta = robot_pose[2]

		# xy coordinates from waypoints
		Vx = waypoints[gotopt][0]
		Vy = waypoints[gotopt][1]

		# calculate desired v and w using feedbackLin
		#desiredV, desiredW = self.feedbackLin(Vx - current_x, Vy - current_y, current_theta, e)
		distance = np.sqrt((Vx - current_x)**2 + (Vy - current_y)**2)
		orient = self.toWholeAngle(np.arctan2(Vy - current_y, Vx - current_x))
		current_theta = self.toWholeAngle(current_theta)
		if abs(orient - current_theta) > 3.0/2.0*np.pi or abs(orient - current_theta) < 1.0/2.0*np.pi:
			desiredV = 0.1*distance
		else:
			desiredV = -0.1*distance
		return (desiredV, 0.0, distance, current_theta)
	

	def limitCmds(self, fwdVel, angVel, maxV, wheel2Center):
		vel_left = fwdVel - wheel2Center * angVel
		vel_right = fwdVel + wheel2Center * angVel
		ratio_L = abs(maxV / vel_left)
		ratio_R = abs(maxV / vel_right)

		if (abs(vel_left) < maxV) and (abs(vel_right) < maxV):
			cmdV = fwdVel
			cmdW = angVel
		else:
			cmdV = fwdVel * min(ratio_L, ratio_R)
			cmdW = fwdVel * min(ratio_L, ratio_R)

		return (cmdV, cmdW)

	def toWholeAngle(self, original_angle):
		if original_angle > 0:
			return original_angle
		else:
			return original_angle + 2*np.pi




if __name__ == '__main__':
	try:
		rospy.init_node('waypointFollower', anonymous=True)

		rosbot = RosbotFollower()

		rospy.spin()
		

	except rospy.ROSInterruptException: pass
#!/usr/bin/python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import tf

class RosbotFollower:
	def __init__(self):
		# set up publisher
		self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

		# set up the subscriber
		self.pose_subscriber = rospy.Subscriber('/pose', PoseStamped, self.retrive_pose)
		self.robot_pose = np.array([0.0,0.0,0.0])
		self.ProbTable = np.array([0.0])#probabilities 
		#self.prev_pose = self.robot_pose

		# wait one second to give subscriber time to connect
		rospy.sleep(1)

	def retrive_pose(self, data):

		#self.robot_pose = data
		self.robot_pose[0] = data.pose.position.x
		self.robot_pose[1] = data.pose.position.y
		orien_x = data.pose.orientation.x
		orien_y = data.pose.orientation.y
		orien_z = data.pose.orientation.z
		orien_w = data.pose.orientation.w
		anglesFromOrien = tf.transformations.euler_from_quaternion([orien_x, orien_y, orien_z, orien_w])
		self.robot_pose[2] = anglesFromOrien[2]
		# print(anglesFromOrien[0], anglesFromOrien[1], anglesFromOrien[2])
		# print(self.robot_pose[0], self.robot_pose[1], self.robot_pose[2])

	def feedbackLin(self, Vx, Vy, theta, e):
		# transform vx and vy into corresponding v and w using feedbackLin
		# linearlization via linear transformations
		R_BI = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])
		# vw_array = np.array([[1, 0], [0, 1/e]]) @ R_BI @ np.array([[Vx], [Vy]])
		vw_array = np.matmul(np.matmul(np.array([[1, 0], [0, 1/e]]), R_BI), np.array([[Vx], [Vy]]))
		vLin = vw_array[0][0] #+ 0.1*(self.robot_pose[0] - self.prev_pose[0])
		wLin = vw_array[1][0]

		return (vLin, wLin)

	def visitWaypoints(self, gotopt, waypoints, e):
		# unpack the data
		# self.prev_pose = self.robot_pose
		current_x = self.robot_pose[0]
		current_y = self.robot_pose[1]
		current_theta = self.robot_pose[2]

		# xy coordinates from waypoints
		Vx = waypoints[gotopt][0]
		Vy = waypoints[gotopt][1]

		# calculate desired v and w using feedbackLin
		desiredV, desiredW = self.feedbackLin(Vx - current_x, Vy - current_y, current_theta, e)
		distance = np.sqrt((Vx - current_x)**2 + (Vy - current_y)**2)

		return (desiredV, desiredW, distance, current_theta)

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


	def take_action(self):
		# initialization
		#waypoints = np.array([[-3/3, 0, 2*np.pi/3], [0, -3/3, 0], [3/3, 0, np.pi/2], [3/3, 0, 170*np.pi/180]])
		# waypoints = np.array([[2.0,0.5,0.0]])
		waypoints = np.array([[2, 0, np.pi/2], [2, 2, np.pi/2]])
		n = len(waypoints) - 1
		gotopt = 0
		closeEnough = 0.30
		e = 0.4
		kp = 2
		maxV = 0.09
		wheel2Center = 23.5/200
		# print(waypoints)


		while not rospy.is_shutdown():
			desiredV, desiredW, distance, current_theta = self.visitWaypoints(gotopt, waypoints, e)
			cmdV, cmdW = self.limitCmds(desiredV, desiredW, maxV, wheel2Center)
			print(gotopt, current_theta)

			# move to next point if the criteria is satisfied
			if distance <= closeEnough and gotopt < n:
			
				if abs(current_theta - waypoints[gotopt][2]) >= 2*np.pi/180:
					desiredV = 0
					if current_theta - waypoints[gotopt][2] > 0:
						desiredW = -abs(current_theta - waypoints[gotopt][2]) * kp
					else:
						desiredW = abs(current_theta - waypoints[gotopt][2])
					cmdV, cmdW = self.limitCmds(desiredV, desiredW)
				else:
					gotopt = gotopt + 1



			# final waypoints
			if distance <= closeEnough and gotopt == n:
				cmdV, cmdW = 0.0, 0.0
			
			vel_msg = Twist()
			vel_msg.linear.x = cmdV
			vel_msg.angular.z = cmdW
			self.velocity_publisher.publish(vel_msg)

			# print(cmdV, cmdW)
			# create Twist message
			#vel_msg = Twist()
			#vel_msg.linear.x = cmdV
			#vel_msg.angular.z = cmdW
			#self.velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
	try:
		rospy.init_node('waypointFollower', anonymous=True)

		rosbot = RosbotFollower()
		rosbot.take_action()
		

	except rospy.ROSInterruptException: pass
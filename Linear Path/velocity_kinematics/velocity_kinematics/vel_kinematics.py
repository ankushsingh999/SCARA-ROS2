#import all the dependencies required to run the node
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from srv_interface.srv import JointVel
from srv_interface.srv import EndeffVel
import numpy as np
import math
import csv

#initilize the robot_fk class by inheriting the Node class from rclpy.node
class robot_pdcontrol(Node):
	#define the constructor for the class
	def __init__(self):
		#call the parent node constructor and give the name to the node
		super().__init__('robot_pdcontrol')
		
		#initilaize current states, reference values, torques
		
		self.q1=0.0
		self.q2=0.0
		self.q3=0.0
		self.q1_dot=0.0
		self.q2_dot=0.0
		self.q3_dot=0.0

		self.prev_e1 = 0.0
		self.prev_e2 = 0.0
		self.prev_e3 = 0.0

		self.u1 = 0.0
		self.u2 = 0.0
		self.u3 = 0.0
		
		self.v1_end = 0.0
		self.v2_end = 0.0
		self.v3_end = 0.0
		
		self.q1_ref=0.0
		self.q2_ref=0.0
		self.q3_ref=0.01
		self.q1_dot_ref=0.0
		self.q2_dot_ref=0.0
		self.q3_dot_ref=0.0
		self.v1_end_ref = 0.0
		self.v2_end_ref = 0.0
		self.v3_end_ref = 0.0
		
		self.t_curr = 0.0
		self.t_prev = 0.0

		#open file and creater writer
		self.f = open('/home/ankush/Desktop/plot.csv','w')
		self.writer = csv.writer(self.f)
		self.writer.writerow(['q1_dot','q2_dot','q3_dot','u1','u2'])
		
		#create subscriber joint_states
		self.sub = self.create_subscription(JointState,'joint_states',self.callback_sub,10)
		
		#create service joint_ref
		self.srv1 = self.create_service(JointVel,'joint_vel_ref',self.callback_srv_jointvel)
		self.srv2 = self.create_service(EndeffVel,'endeff_vel_ref',self.callback_srv_endeffvel)
		
		
		#create effort publisher
		self.pub = self.create_publisher(Float64MultiArray,'forward_effort_controller/commands',10)
		
	#define callback function for forward kinematics

	def callback_sub(self,msg):
		#self.t_curr = msg.header.stamp.sec
		#print(self.t_curr)
		#ref end effector velocity
		self.v1_end_ref = 0.0
		self.v2_end_ref = 1.0
		self.v3_end_ref = 0.0
		#current joint states
		self.q1 = msg.position[0]
		self.q2 = msg.position[1]
		self.q3 = msg.position[2]
		self.q1_dot = msg.velocity[0]
		self.q2_dot = msg.velocity[1]
		self.q3_dot = msg.velocity[2]
		
		jacobian_mtx = np.array([[-(math.sin(self.q1)*math.cos(self.q2) + math.sin(self.q1) + math.cos(self.q1)*math.sin(self.q2)), -(math.sin(self.q1)*math.cos(self.q2) + math.cos(self.q1)*math.sin(self.q2)), 0],
					[math.cos(self.q1)*math.cos(self.q2) + math.cos(self.q1) - math.sin(self.q1)*math.sin(self.q2), math.cos(self.q1)*math.cos(self.q2) - math.sin(self.q1)*math.sin(self.q2),0],
					[0,0,1],
					[0,0,0],
					[0,0,0],
					[1,1,0]],dtype=float)
		q_dot_array = np.array([[self.v1_end_ref], [self.v2_end_ref], [self.v3_end_ref]],dtype=float)
		q_dot = np.matmul(np.linalg.pinv(jacobian_mtx[0:3,0:3]), q_dot_array)
		self.q1_dot_ref = q_dot[0,0]
		self.q2_dot_ref = q_dot[1,0]
		self.q3_dot_ref = q_dot[2,0]
		
	        ##if self.q1_ref == 0 and self.q2_ref == 0 and self.q3_ref == 0 :
	        ##self.u1 = 0
	        ##self.u2 = 0 
	        ##self.u3 = 0
	        ##
	        ##else  
	        
		#joint1 control
		kp1 = 0.5 # <0.9
		kd1 = 0.05 # <0.8
		e1 = self.q1_dot-self.q1_dot_ref
		e1_dot = 100*(e1 - self.prev_e1)
		#e1_dot = (e1 - self.prev_e1)/(self.t_curr-self.t_prev)
		self.u1 = -kp1*e1-kd1*e1_dot
		self.prev_e1 = e1
		
		#joint2 controln endeff_vel_ref ^print(self.u1)C

		kp2 = 0.75
		kd2 = 0.08
		e2 = self.q2_dot - self.q2_dot_ref
		e2_dot = 100*(e2 - self.prev_e2)
		#e2_dot = (e2 - self.prev_e2)/(self.t_curr-self.t_prev)
		self.u2 = -kp2*e2-kd2*e2_dot
		self.prev_e2 = e2
		
		#joint3 control
		kp3 = 1
		kd3 = 1
		e3 = self.q3_dot - self.q3_dot_ref
		e3_dot = 100*(e3 - self.prev_e3)
		#e3_dot = (e3 - self.prev_e3)/(self.t_curr-self.t_prev)
		self.u3 = -kp3*e3-kd3*e3_dot
		self.prev_e3 = e3
		
		print(self.u1)
		print(self.u2)
		print(self.u3)
		
		#publish control inputs
		torq = Float64MultiArray()
		torq.data = [self.u1,self.u2,self.u3]
		self.pub.publish(torq)
		end_eff = np.matmul(jacobian_mtx,np.array([[self.q1_dot],[self.q2_dot],[self.q3_dot]]))
		self.v1_end = end_eff[0,0]
		self.v2_end = end_eff[1,0]
		self.v3_end = end_eff[2,0]
		data = [self.v1_end,self.v2_end,self.v3_end,self.u1,self.u1]
		self.writer.writerow(data)
		self.t_prev = self.t_curr
		# q1 --> q1_dot_ref = end effector ref	velocities			

	def callback_srv_jointvel(self,request,response):
		l1 = 1
		l2 = 1
		l3 = 1
		jacobian_mtx = np.array([[-(math.sin(self.q1)*math.cos(self.q2) + math.sin(self.q1) + math.cos(self.q1)*math.sin(self.q2)), -(math.sin(self.q1)*math.cos(self.q2) + math.cos(self.q1)*math.sin(self.q2)), 0],
					[math.cos(self.q1)*math.cos(self.q2) + math.cos(self.q1) - math.sin(self.q1)*math.sin(self.q2), math.cos(self.q1)*math.cos(self.q2) - math.sin(self.q1)*math.sin(self.q2),0],
					[0,0,1],
					[0,0,0],
					[0,0,0],
					[1,1,0]],dtype=float)
		q_dot_array = np.array([[request.v1], [request.v2], [request.v3]],dtype=float)
		print(jacobian_mtx)
		q_dot = np.matmul(jacobian_mtx, q_dot_array)
		response.ev1 = q_dot[0,0]
		response.ev2 = q_dot[1,0]
		response.ev3 = q_dot[2,0]
		self.v1_end_ref = q_dot[0,0]
		self.v1_end_ref = q_dot[1,0]
		self.v1_end_ref = q_dot[2,0]
		return response
	
	def callback_srv_endeffvel(self,request,response):
		l1 = 1
		l2 = 1
		l3 = 1
		
		jacobian_mtx = np.array([[-(math.sin(self.q1)*math.cos(self.q2) + math.sin(self.q1) + math.cos(self.q1)*math.sin(self.q2)), -(math.sin(self.q1)*math.cos(self.q2) + math.cos(self.q1)*math.sin(self.q2)), 0],
					[math.cos(self.q1)*math.cos(self.q2) + math.cos(self.q1) - math.sin(self.q1)*math.sin(self.q2), math.cos(self.q1)*math.cos(self.q2) - math.sin(self.q1)*math.sin(self.q2),0],
					[0,0,1],
					[0,0,0],
					[0,0,0],
					[1,1,0]],dtype=float)
		q_dot_array = np.array([[request.ev1], [request.ev2], [request.ev3]],dtype=float)
		q_dot = np.matmul(np.linalg.pinv(jacobian_mtx)[:,0:3], q_dot_array) #inverse kinematics
		response.v1 = q_dot[0,0]
		response.v2 = q_dot[1,0]
		response.v3 = q_dot[2,0]
		self.q1_dot_ref = q_dot[0,0]
		self.q2_dot_ref = q_dot[1,0]
		self.q3_dot_ref = q_dot[2,0]
		return response
	
	def close_file(self):
		self.f.close()

#defining the main function
def main(args=None):
	#initialize rclpy and create node
	rclpy.init(args=args)
	robo_ctrl = robot_pdcontrol()
	rclpy.spin(robo_ctrl)
	#destroy node
	robo_ctrl.destroy_node()
	robo_ctrl.close_file()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()

import numpy as np
import pdb
import matplotlib.pyplot as plt
import matplotlib
import copy
from itertools import cycle
from pprint import pprint
# from ROB541_GeometricMechanics.HW1 import triangle
colors = cycle(['r', 'b', 'g'])

class GeoOps(object):
	@staticmethod
	def transformation_matrix(transform):
		# transform: x, y, theta
		c, s = np.cos(transform[2]), np.sin(transform[2])
		T = np.array([ [c,-s,transform[0]],
						[s,c,transform[1]],
						[0,0,1]])
		return T

	@staticmethod
	def pose_from_transformation_matrix(transformation_matrix):
		pose = np.zeros(3)
		pose[0] = transformation_matrix[0,2]
		pose[1] = transformation_matrix[1,2]
		pose[2] = np.arctan2(transformation_matrix[1,0], transformation_matrix[0,0])
		return pose

	@classmethod
	def right_action(cls, current_pose, relative_pose):
		combined_transformation = np.dot(cls.transformation_matrix(current_pose), cls.transformation_matrix(relative_pose))
		pose = cls.pose_from_transformation_matrix(combined_transformation)
		return pose
	
	@classmethod
	def inverse_action(cls, pose):
		# undoing a transform = undoing translation and then undoing rotation
		transl_inv = cls.transformation_matrix([-pose[0], -pose[1], 0])
		rot_inv = cls.transformation_matrix([0,0,pose[2]])
		T = np.dot(transl_inv, rot_inv)
		p = cls.pose_from_transformation_matrix(T)
		return p

	@staticmethod
	def adjoint(pose):
		c = np.cos
		s = np.sin
		x,y,theta = pose[:]
		ad = [	[c(theta), -s(theta),	y],
				[s(theta), c(theta),	-x],
				[0,			0,			1]
			]
		return ad

	@staticmethod
	def adjoint_inv(pose):
		x,y,theta = pose[:]
		c = np.cos(theta)
		s = np.sin(theta)
		ad_inv = [	[c,		s,	x*s-y*c],
					[-s,	c,	x*c+y*s],
					[0,		0,	1]
				]
		return np.array(ad_inv)

	@staticmethod
	def right_lifted(pose):
		x,y,theta = pose[:]
		c = np.cos(theta)
		s = np.sin(theta)
		act = [		[1,	0,	-y],
					[0,	1,	x],
					[0,	0,	1]
			]
		return np.array(act)

	@staticmethod
	def right_lifted_inv(pose):
		x,y,theta = pose[:]
		c = np.cos(theta)
		s = np.sin(theta)
		act = [		[1,	0,	y],
					[0,	1,	-x],
					[0,	0,	1]
			]
		return np.array(act)



class Comprehension(object):
	def __init__(self):
		i = 1



class drawarm(object):
	def __init__(self):
		self.ax = plt.subplot(1,1,1)
		self.draw_obj = []
		self.c = None

	def add_arm(self, arm):
		self.arm = arm

	def draw(self, arm):
		self.removeDrawObjects()
		# pass in arm object
		p = np.array([0,0,arm.alpha[0]]).reshape(3,1)
		poses = arm.get_poses()
		arrow_poses = arm.get_arrowposes()
		vel_pt = arm.calc_point_vel(3, arm.h[-1])

		for p_prox, p_dist, p_arrow in zip(poses[:-1], poses[1:], arrow_poses):
			self.drawLink(p_prox, p_dist)
			self.drawArrow(p_arrow)
		self.formatPlot()

	def drawLink(self, p_prox, p_dist):
		self.c = next(colors)
		l = matplotlib.lines.Line2D([p_prox[0], p_dist[0]],[p_prox[1], p_dist[1]], 
			color = self.c, markeredgecolor = self.c)
		c_prox = plt.Circle(p_prox, 0.01, color='r',fill='b')
		c_dist = plt.Circle(p_dist, 0.02, color='r',fill='b')
		self.ax.add_patch(c_prox)
		self.ax.add_patch(c_dist)
		self.ax.add_line(l)
		self.draw_obj.append(l)
		self.draw_obj.append(c_prox)
		self.draw_obj.append(c_dist)

	def formatPlot(self):
		self.ax.set_aspect('equal')
		self.ax.set_xlim(-5,5)
		self.ax.set_ylim(-5,5)

	def removeDrawObjects(self):
		[o.remove() for o in self.draw_obj]
		self.draw_obj = []

	def drawArrow(self, p_arrow):
		arrow = plt.Arrow(p_arrow[0], p_arrow[1], 1, 1, lw = 1, fill=False, width = 0.5, color = self.c)
		self.ax.add_patch(arrow)
		self.draw_obj.append(arrow)

class arm(object):
	def __init__(self, a_local, h):
		self.joint_count = len(a_local)
		self.a_local = a_local  # joint defined as relative movement to next joint
		self.h = h # defines the transformations from proximal to distal point
		self.alpha = np.zeros(self.joint_count)
		self.base_pose = np.array([0,0,0])
		self.poses = self.calc_poses(self.alpha)

	def set_joints(self, alpha):
		# set the joint values
		self.alpha = alpha

	def get_poses(self):
		self.poses = self.calc_poses(self.alpha)
		return self.poses

	def calc_poses(self, alpha):
		pose = [copy.deepcopy(self.base_pose)]
		for j in range(self.joint_count):
			pose_next = GeoOps.right_action(pose[-1], self.a_local[j]*alpha[j])
			pose_next = GeoOps.right_action(pose_next, self.h[j])
			pose.append(copy.deepcopy(pose_next))
		return pose

	def set_arrow_points(self, h_arrow):
		# sets point where the arrow will attach to
		self.h_arrow = h_arrow

	def calc_arrowposes(self):
		pose = []
		for j in range(0,self.joint_count):
			pose_next = GeoOps.right_action(self.poses[j+1], GeoOps.inverse_action(self.h_arrow[j]))
			pose.append(copy.deepcopy(pose_next))
		return pose

	def get_arrowposes(self):
		self.arrow_poses = self.calc_arrowposes()
		return self.arrow_poses

	def set_joint_vel(self, alpha_dot):
		self.alpha_dot = alpha_dot

	def calc_point_vel(self, link_num, h):
		#calculates the world velocity at a point on a link
		J_spatial = self.calc_Jacobian_spatial()
		p = self.poses(link_num+1)
		TeRg = GeoOps.right_lifted(p)
		J = np.dot(TeRg, J_spatial)
		v = np.dot(J, self.alpha_dot)
		pdb.set_trace()
		return v

	def calc_Jacobian_spatial(self):
		# calcualtes the jacobian using the spatial approach
		J_spatial = []
		for j in range(self.joint_count):
			J_spatial.append(np.dot(GeoOps.adjoint(self.poses[j+1]), self.a_local[j]))
			# pdb.set_trace()
		J_spatial = np.transpose(np.array(J_spatial))
		return J_spatial

	def Jacobian_spatial_to_world(self, pose):
		J_spatial = self.calc_Jacobian_spatial()
		TeRg = GeoOps.right_lifted(pose)
		J = np.dot(TeRg, J_spatial)

class HW3(object):
	def RRP(self):
		alpha = [np.pi/2, np.pi/2, 1]
		alpha_dot = np.array([np.pi/10, np.pi/10, 1/10])
		T = 5

		a1 = np.array([0,0,1]) # rotary
		h1 = np.array([2,0,0])
		h1_mp = h1/2

		a2 = np.array([0,0,1]) # rotary
		h2 = np.array([1,0,0])
		h2_mp = h2/2

		a3 = np.array([1,0,0]) # prismatic
		h3 = np.array([alpha[2],0,0])
		h3_mp = h3/2

		a = np.vstack((a1, a2, a3))
		h = np.vstack((h1, h2, h3))
		h_mp = np.vstack((h1_mp, h2_mp, h3_mp))
		A = arm(a, h)
		A.set_joints(alpha)
		A.set_arrow_points(h_mp)
		D = drawarm()
		A.calc_arrowposes()
		plt.ion()
		t_last = 0.0
		for t in np.arange(0, T, 0.1):
			dt = t-t_last
			alpha += dt*alpha_dot
			A.set_joints(alpha)
			D.draw(A)
			plt.draw()
			plt.pause(0.1)
		plt.show()







if __name__ == '__main__':

	hw3 = HW3()
	hw3.RRP()

	# alpha = [np.pi/2, np.pi/2, 1]

	# a1 = np.array([0,0,1]) # rotary
	# h1 = np.array([2,0,0])
	# h1_mp = h1/2

	# a2 = np.array([0,0,1]) # rotary
	# h2 = np.array([1,0,0])
	# h2_mp = h2/2

	# a3 = np.array([1,0,0]) # prismatic
	# h3 = np.array([alpha[2],0,0])
	# h3_mp = h3/2

	# a = np.vstack((a1, a2, a3))
	# h = np.vstack((h1, h2, h3))
	# h_mp = np.vstack((h1_mp, h2_mp, h3_mp))
	# A = arm(a, h)
	# A.set_joints(alpha)
	# A.set_arrow_points(h_mp)
	# D = drawarm()
	# A.calc_arrowposes()
	# plt.ion()
	# for ia in range(len(alpha)):
	# 	for i in np.arange(0, np.pi, 0.1):
	# 		alpha[ia] += 0.1
	# 		D.draw(A)
	# 		plt.draw()
	# 		plt.pause(0.1)
	# plt.show()

import numpy as np
import pdb
import matplotlib.pyplot as plt
import matplotlib
import copy
from itertools import cycle
from pprint import pprint
import seaborn as sns
sns.set(style="whitegrid")
from matplotlib.animation import FuncAnimation
# from ROB541_GeometricMechanics.HW1 import triangle
colors = ['r', 'b', 'g']
linestyle = cycle(('-', ':', '-.', '--')) 



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
		rot_inv = cls.transformation_matrix([0,0,-pose[2]])
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

class drawarm(object):
	def __init__(self):
		self.ax = plt.subplot(1,1,1)
		self.draw_obj = []
		self.c = None
		self.arrow_scale = 2
		self.colors = None

	def add_arm(self, arm):
		self.arm = arm

	def draw(self, arm):
		# pass in arm object
		p_prox = arm.poses[:-1]
		p_dist = arm.poses[1:]
		p_arrow = arm.arrow_poses
		vs = arm.vs
		if self.colors is None:
			self.colors = cycle(colors[:arm.joint_count])
		self.removeDrawObjects()
		[self.drawLink(p_p,p_d) for p_p,p_d in zip(p_prox,p_dist)]
		[self.drawArrow(p_a, vel_pt) for p_a,vel_pt in zip(p_arrow,vs)]
		self.formatPlot()

	def drawLink(self, p_prox, p_dist):
		self.c = next(self.colors)
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

	def drawArrow(self, p_arrow, v_arrow):
		self.c = next(self.colors)
		v_arrow *= self.arrow_scale
		if abs(v_arrow[0]) > 1e-10 or abs(v_arrow[1]) > 1e-10:
			arrow = plt.Arrow(p_arrow[0], p_arrow[1], v_arrow[0], v_arrow[1], lw = 1, fill=False, width = 0.5, color = self.c)
			self.ax.add_patch(arrow)
			self.draw_obj.append(arrow)

class arm(object):
	def __init__(self, a_local, h):
		self.joint_count = len(a_local)
		self.a_local = a_local  # joint defined as relative movement to next joint
		self.h = h # defines the transformations from proximal to distal point
		self.alpha = np.zeros(self.joint_count) # joint positions
		self.alpha_dot = None # joint velocity
		self.base_pose = np.array([0,0,0]) # location of base of robot
		self.poses = self.calc_poses(self.alpha) # pose in world space
		self.h_arrow = None
		self.vs = None

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
		# determines the pose of the arrow locations based on the pose of the arm
		pose = []
		for j in range(0,self.joint_count):
			# thsi should be fixed.  this is actually the distance from the distal end
			pose_next = GeoOps.right_action(self.poses[j+1], GeoOps.inverse_action(self.h_arrow[j]))
			pose.append(copy.deepcopy(pose_next))
		return pose

	def get_arrowposes(self):
		self.arrow_poses = self.calc_arrowposes()
		return self.arrow_poses

	def set_joint_vel(self, alpha_dot):
		self.alpha_dot = alpha_dot

	def calc_point_vel(self, link_num, h):
		# calculates the world velocity at a point on a link
		# link number should be from zero (not start at 1)
		J_spatial_all = self.calc_Jacobian_spatial() # full Jacobian
		J_spatial = J_spatial_all[:, 0:link_num+1] # relevant Jacobian
		p = self.poses[link_num+1]
		poi = GeoOps.right_action(p, GeoOps.inverse_action(h))
		TeRg = GeoOps.right_lifted(poi)
		J = np.dot(TeRg, J_spatial)
		v = np.dot(J, self.alpha_dot[0:link_num+1])
		return v

	def calc_Jacobian_spatial(self):
		# calcualtes the jacobian using the spatial approach
		J_spatial = []
		for j in range(self.joint_count):
			J_spatial.append(np.dot(GeoOps.adjoint(self.poses[j]), self.a_local[j]))
		J_spatial = np.transpose(np.array(J_spatial))
		return J_spatial

	def Jacobian_spatial_to_world(self, pose):
		J_spatial = self.calc_Jacobian_spatial()
		TeRg = GeoOps.right_lifted(pose)
		J = np.dot(TeRg, J_spatial)

	def move(self):
		vs = []
		poses = self.get_poses()
		arrow_poses = self.get_arrowposes()
		for i in range(self.joint_count):
			p_prox, p_dist = poses[i:i+2]
			p_arrow = arrow_poses[i]
			vel_pt = self.calc_point_vel(i, self.h_arrow[i])
			vs.append(copy.deepcopy(vel_pt))
		self.vs = vs
		return poses[:-1], poses[1:], arrow_poses, vs


class HW3(object):
	def RR_test(self):
		# alpha = [np.pi/2, np.pi]
		# alpha = [0, np.pi]
		alpha = [0, 0]
		T = 10
		time_traj = np.arange(0, T, 0.1)
		alpha_dot_traj = [np.array([np.pi/5, 0]) for t in time_traj]
		# alpha_dot_traj = [np.array([0, 0]) for t in time_traj]
		traj = zip(time_traj, alpha_dot_traj)

		a1 = np.array([0,0,1]) # rotary
		h1 = np.array([2,0,0])
		h1_mp = h1/2

		a2 = np.array([0,0,1]) # rotary
		h2 = np.array([2,0,0])
		h2_mp = h2/2

		a = np.vstack((a1, a2))
		h = np.vstack((h1, h2))
		h_mp = np.vstack((h1_mp, h2_mp))
		A = arm(a, h)
		A.set_joints(alpha)
		A.set_arrow_points(h_mp)
		D = drawarm()
		A.calc_arrowposes()
		plt.ion()
		v_all = []
		t_last = 0.0
		for t, adot in traj:
			dt = t-t_last
			alpha += dt*adot
			A.set_joints(alpha)
			A.set_joint_vel(adot)
			p_prox, p_dist, a_poses, vs = A.move()
			D.draw(A)
			v_all.append(copy.deepcopy(vs))
			plt.draw()
			plt.pause(0.01)
			t_last = t
		plt.show()
		v_all = np.array(v_all)
		plt.figure()
		plt.plot(time_traj, v_all[:,0,0], ls = '-', label = 'L0_X')
		plt.plot(time_traj, v_all[:,0,1], ls = '-', label = 'L0_Y')
		plt.plot(time_traj, v_all[:,1,0], ls = ':', label = 'L1_X')
		plt.plot(time_traj, v_all[:,1,1], ls = ':', label = 'L1_Y')
		plt.legend()
		plt.draw()
		pdb.set_trace()

	def RRP_test(self):
		alpha = [0, 0, 1]
		T = 10
		time_traj = np.arange(0, T, 0.1)
		alpha_dot_traj = [np.array([np.pi/5, np.pi/5, np.cos(t)/10*0]) for t in time_traj]
		traj = zip(time_traj, alpha_dot_traj)

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
		v_all = []
		t_last = 0.0
		for t, adot in traj:
			dt = t-t_last
			alpha += dt*adot
			A.set_joints(alpha)
			A.set_joint_vel(adot)
			p_prox, p_dist, a_poses, vs = A.move()
			D.draw(A)
			v_all.append(copy.deepcopy(v))
			plt.draw()
			plt.pause(0.1)
			t_last = t
		v_all = np.array(v_all)
		plt.figure()
		for i in range(A.joint_count):
			ls = next(linestyle)
			plt.plot(time_traj, v_all[:,i,0], ls = ls, label = 'L%s_X' %i)
			plt.plot(time_traj, v_all[:,i,1], ls = ls, label = 'L%s_Y' %i)
		plt.legend()
		plt.draw()
		plt.show()
		pdb.set_trace()

	def RPR_test(self):
		alpha = [0, 1,  0]
		T = 10
		time_traj = np.arange(0, T, 0.1)
		alpha_dot_traj = [np.array([np.pi/5, np.cos(t)/10,  np.pi/5]) for t in time_traj]
		traj = zip(time_traj, alpha_dot_traj)

		a1 = np.array([0,0,1]) # rotary
		h1 = np.array([2,0,0])
		h1_mp = h1/2

		a2 = np.array([1,0,0]) # prismatic
		h2 = np.array([alpha[1],0,0])
		h2_mp = h2/2

		a3 = np.array([0,0,1]) # rotary
		h3 = np.array([1,0,0])
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
		v_all = []
		t_last = 0.0
		for t, adot in traj:
			dt = t-t_last
			alpha += dt*adot
			A.set_joints(alpha)
			A.set_joint_vel(adot)
			p_prox, p_dist, a_poses, vs = A.move()
			D.draw(A)
			v_all.append(copy.deepcopy(v))
			plt.draw()
			plt.pause(0.01)
			t_last = t
		v_all = np.array(v_all)
		plt.figure()
		for i in range(A.joint_count):
			ls = next(linestyle)
			plt.plot(time_traj, v_all[:,i,0], ls = ls, label = 'L%s_X' %i)
			plt.plot(time_traj, v_all[:,i,1], ls = ls, label = 'L%s_Y' %i)
		plt.legend()
		plt.draw()
		plt.show()
		pdb.set_trace()

	def RRP_image(self):
		alpha = [np.deg2rad(-45), np.deg2rad(135), 2]
		T = 10
		time_traj = np.arange(0, T, 0.1)
		alpha_dot = [0,0,0]

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
		A.set_joint_vel(alpha_dot)
		D = drawarm()
		A.move()
		D.draw(A)
		D.ax.set_title('RRP Configuration')
		D.ax.get_figure().savefig('RRP.png')
		plt.close(D.ax.get_figure())

	def RPR_image(self):
		alpha = [np.deg2rad(30), 0.5, np.deg2rad(135)]
		T = 10
		time_traj = np.arange(0, T, 0.1)
		alpha_dot = [0,0,0]

		a1 = np.array([0,0,1]) # rotary
		h1 = np.array([2,0,0])
		h1_mp = h1/2

		a2 = np.array([1,0,0]) # prismatic
		h2 = np.array([alpha[1],0,0])
		h2_mp = h2/2

		a3 = np.array([0,0,1]) # rotary
		h3 = np.array([1,0,0])
		h3_mp = h3/2

		a = np.vstack((a1, a2, a3))
		h = np.vstack((h1, h2, h3))
		h_mp = np.vstack((h1_mp, h2_mp, h3_mp))
		A = arm(a, h)
		A.set_joints(alpha)
		A.set_arrow_points(h_mp)
		A.set_joint_vel(alpha_dot)
		D = drawarm()
		A.move()
		D.draw(A)
		D.ax.set_title('RPR Configuration')
		D.ax.get_figure().savefig('RPR.png')
		plt.close(D.ax.get_figure())

	def RRP_animation(self):
		alpha = [0, np.pi, 1]
		T = 10
		time_traj = np.arange(0, T, 0.1)
		alpha_dot_traj = np.array([np.array([np.pi/5, -np.pi/5, np.cos(t)/10]) for t in time_traj])
		alpha_dot_traj[-int(len(time_traj)/2):,2] *= -1
		traj = zip(time_traj, alpha_dot_traj)

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
		t_last = 0.0
		for t, adot in traj:
			dt = t-t_last
			alpha += dt*adot
			A.set_joints(alpha)
			A.set_joint_vel(adot)
			A.move()
			D.draw(A)
			plt.draw()
			t_last = t
			yield D.ax.get_figure()

	def RPR_animation(self):
		alpha = [0, 1,  0]
		T = 10
		time_traj = np.arange(0, T, 0.1)
		alpha_dot_traj = np.array([np.array([np.pi/5, 0.25,  np.pi]) for t in time_traj])
		alpha_dot_traj[-int(len(time_traj)/2):,1] *= -1
		traj = zip(time_traj, alpha_dot_traj)

		a1 = np.array([0,0,1]) # rotary
		h1 = np.array([2,0,0])
		h1_mp = h1/2

		a2 = np.array([1,0,0]) # prismatic
		h2 = np.array([alpha[1],0,0])
		h2_mp = h2/2

		a3 = np.array([0,0,1]) # rotary
		h3 = np.array([1,0,0])
		h3_mp = h3/2

		a = np.vstack((a1, a2, a3))
		h = np.vstack((h1, h2, h3))
		h_mp = np.vstack((h1_mp, h2_mp, h3_mp))
		A = arm(a, h)
		A.set_joints(alpha)
		A.set_arrow_points(h_mp)
		D = drawarm()
		t_last = 0.0
		for t, adot in traj:
			dt = t-t_last
			alpha += dt*adot
			A.set_joints(alpha)
			A.set_joint_vel(adot)
			A.move()
			D.draw(A)
			D.ax.set_xlim(-8,8)
			D.ax.set_ylim(-8,8)
			plt.draw()
			t_last = t
			yield D.ax.get_figure()

	def record_gif(self, gen_func, savefn):
		gen = gen_func()
		frame_func = lambda t: next(gen)
		ax = plt.subplot(1,1,1)
		f = ax.get_figure()
		animation = FuncAnimation(f, frame_func, frames = np.arange(0,10,0.1), interval = 200)
		animation.save(savefn + '.gif', dpi = 80, writer = 'imagemagick')











if __name__ == '__main__':

	hw3 = HW3()
	# hw3.RR_test()
	# hw3.RRP_test()
	# hw3.RPR_test()
	# hw3.RRP_image()
	# hw3.RPR_image()
	hw3.record_gif(hw3.RRP_animation, 'RRP')
	hw3.record_gif(hw3.RPR_animation, 'RPR')

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

import numpy as np
import pdb
import matplotlib.pyplot as plt
import matplotlib
import copy
# from ROB541_GeometricMechanics.HW1 import triangle


def transformation_matrix(transform):
	# transform: x, y, theta
	c, s = np.cos(transform[2]), np.sin(transform[2])
	T = np.array([ [c,-s,transform[0]],
					[s,c,transform[1]],
					[0,0,1]])
	return T

def pose_from_transformation_matrix(transformation_matrix):
	pose = np.zeros(3)
	pose[0] = transformation_matrix[0,2]
	pose[1] = transformation_matrix[1,2]
	pose[2] = np.arctan2(transformation_matrix[1,0], transformation_matrix[0,0])
	return pose

class Comprehension(object):
	def __init__(self):
		i = 1



class drawarm(object):
	def __init__(self):
		self.ax = plt.subplot(1,1,1)
		self.draw_obj = []

	def add_arm(self, arm):
		self.arm = arm



	def draw(self, arm):
		self.removeDrawObjects()
		# pass in arm object
		p = np.array([0,0,arm.alpha[0]]).reshape(3,1)
		for j in range(arm.joint_count):
			h = arm.h[j] + arm.a_local[j]*arm.alpha[j] # local transform to end of link
			# pdb.set_trace()	
			T_end = np.dot(transformation_matrix(p), transformation_matrix(h)) #right action
			p_end = pose_from_transformation_matrix(T_end)
			self.drawLink(p, p_end)
			p = copy.deepcopy(p_end)
		self.formatPlot()
		# plt.show()
		# pdb.set_trace()

	def drawLink(self, p_prox, p_dist):
		l = matplotlib.lines.Line2D([p_prox[0], p_dist[0]],[p_prox[1], p_dist[1]], 
			markeredgecolor = 'r')
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




class arm(object):
	def __init__(self, a_local, h):
		self.joint_count = len(a_local)
		self.a_local = a_local  # joint defined as relative movement to next joint
		self.h = h # defines the transformations from proximal to distal point

	def set_joints(self, alpha):
		# set the joint values
		self.alpha = alpha










if __name__ == '__main__':

	alpha = [np.pi/2, np.pi/2, 1]

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
	A = arm(a, h)
	A.set_joints(alpha)
	D = drawarm()
	# D.add_arm(A)
	plt.ion()
	for ia in range(len(alpha)):
		for i in np.arange(0, np.pi, 0.1):
			alpha[ia] += 0.1
			D.draw(A)
			plt.draw()
			plt.pause(0.1)
	D.drawLink(np.pi/2)
	plt.show()

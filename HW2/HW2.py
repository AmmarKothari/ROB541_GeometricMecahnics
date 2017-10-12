import numpy as np
import pdb
import matplotlib.pyplot as plt
from pprint import pprint


def transformation_matrix(pose):
	# transform: x, y, theta
	c, s = cos(pose[2]), sin(pose[2])
	T = np.array([ [c,-s,pose[0]],
					[s,c,pose[1]],
					[0,0,1]])
	return T

def inverse_transformation_matrix(pose):
	T = transformation_matrix(pose)
	T_inv = np.linalg.inv(T)
	return T_inv

def pose_from_transformation(transformation_matrix):
	pose = np.zeros(3)
	pose[0] = transformation_matrix[0,2]
	pose[1] = transformation_matrix[1,2]
	pose[2] = np.arctan2(transformation_matrix[0,1], transformation_matrix[0,0])
	return pose



def inverse_pose(pose):
	inv_pose = np.copy(pose)
	# inv_pose = np.zeros_like(pose)
	inv_pose[0] *= -1
	inv_pose[1] *= -1
	inv_pose[2] *= -1
	return inv_pose


pi = np.pi
cos = np.cos
sin = np.sin

g = [1,1,-pi/4]
g_dot = np.array([1,0,1])

# g circ right == body velocity
g_circ_right = np.dot(inverse_transformation_matrix(g), transformation_matrix(g_dot))

g1 = [0,0,0]
g2 = [-1,1,pi/2]
g3 = [-1,-1,-pi/4]

# gs = [g1, g2, g3]
gs = [g, g1]
gs_dot = [np.dot(transformation_matrix(g_i), g_circ_right) for g_i in gs]

plt.plot(g[0], g[1], 'bo', markersize = 20)
[plt.plot(x[0], x[1], 'go', markersize = 20) for x in gs]

t_last = 0
for t in np.arange(0,1,0.01):
	t_delta = t - t_last
	t_last = t
	for i,g_i in enumerate(gs):
		gs[i] = pose_from_transformation(transformation_matrix(g_i) + gs_dot[i]*t_delta)
	[plt.plot(g_i[0], g_i[1], 'ro') for g_i in gs]
	gs_dot = [np.dot(transformation_matrix(g_i), g_circ_right) for g_i in gs]
	plt.draw()
	plt.pause(0.001)

plt.show()

pdb.set_trace()
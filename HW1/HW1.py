#!/usr/bin/python
# coding=utf-8

'''
1. Create 3 × 3 matrix representations of group elements from their (x, y, θ) parameters.

Operation		Transformation
---------		-------------------------
x				cos(theta)	-sin(theta)	x
y			=	sin(theta)	cos(theta)	y
theta			0			0			1


2. Compose group elements to produce new group elements.

Operation					Transformation
----------------------		--------------------------------
x			u				Operation1 * Operation2
y		+	v			=
theta		gamma


3. Multiply group elements by point locations to get the global positions of points in a
local frame.



'''




import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle, Arc
from matplotlib.collections import PatchCollection
import pdb
import matplotlib

class triangle(object):
	# this is the object that will be rotated
	def __init__(self,f,ax):
		self.f, self.ax = f,ax
		self.patches = []
		self.triangle_pose = [0,0,0]
		self.large_vertices = np.array([[-1,-1], [-1, 1], [1, 0]])
		self.small_vertices = np.array([[0,1],[0,-1],[1,0]])

		self.large_vertices_base = np.array([[-1,-1], [-1, 1], [1, 0]])
		self.small_vertices_base = np.array([[0,1],[0,-1],[1,0]])

		self.circle_color = [0.5, 0.5, 0.5]
		self.large_color  = [1,0,1]
		self.small_color = [0,1,1]

	def rotation_matrix(self, theta):
		c, s = np.cos(theta), np.sin(theta)
		R = np.matrix([[c, -s], [s, c]])
		return R

	def transformation_matrix(self, transform):
		# transform: x, y, theta
		c, s = np.cos(transform[2]), np.sin(transform[2])
		T = np.array([ [c,-s,transform[0]],
						[s,c,transform[1]],
						[0,0,1]])
		return T


	def large_triangle_pose(self, pose):
		R = self.rotation_matrix(pose(2))
		self.large_vertices = np.dot(R,self.large_vertices_base)

	def large_triangle(self):
		pdb.set_trace()
		large_triangle = Polygon(self.large_vertices, True, alpha=0.4, color = self.large_color)
		self.patches.append(large_triangle)

	def small_triangle(self):
		small_triangle = Polygon(self.small_vertices, True, alpha=0.4, color = self.small_color)
		self.patches.append(small_triangle)

	def center(self):
		center = Circle((self.triangle_pose[:2]),radius=0.1, color = self.circle_color, edgecolor = 'auto', alpha=0.4)
		self.patches.append(center)

	def draw(self):
		p = PatchCollection(self.patches, alpha=0.4, match_original=True)
		self.ax.add_collection(p)
		plt.xlim(-5, 5)
		plt.ylim(-5, 5)

	def transform(self, current_pose, transform):
		CT = self.transformation_matrix(current_pose)
		T = self.transformation_matrix(transform)
		new_T = np.dot(CT,T)
		new_pose = new_T[:,2]
		new_pose[2] = current_pose[2] + transform[2]
		return new_pose


	def move_triangle(self, current_pose, transform):
		self.triangle_pose = self.transform(current_pose, transform)
		self.large_vertices = self.vertices_relative_pose(self.large_vertices_base, self.triangle_pose)
		self.small_vertices = self.vertices_relative_pose(self.small_vertices_base, self.triangle_pose)
		self.add_patches()

	def vertices_relative_pose(self, vertices, pose):
		new_vertices = []
		for v in vertices:
			v_pose = [v[0], v[1], 1]	
			new_vertices.append(self.transform(v_pose, pose)[:2])
		return new_vertices

	def add_patches(self):
		self.patches = []
		self.large_triangle()
		self.small_triangle()
		self.center()
		self.draw()



class motion_path(object):
	def __init__(self,f,ax):
		self.f, self.ax = f,ax
		self.patches = []

	def motion_path(self):
		arc1 = Arc((0,1), width=2, height=2, angle=0.0, theta1=-90.0, theta2=90.0)
		self.patches.append(arc1)
		arc2 = Arc((0,3), width=2, height=2, angle=180.0, theta1=-90.0, theta2=90.0)
		self.patches.append(arc2)

	def motion_path_pts(self, pts):
		r = 1
		pts_all = []
		theta_start = 0; theta_end = np.pi
		c = np.array([0,1])
		for theta in np.linspace(theta_start, theta_end, int(pts/2)):
			p = c + np.array([r*np.sin(theta), -r*np.cos(theta)])
			p = np.append(p, theta)
			pts_all.append(p)
		theta_start = 0; theta_end = -np.pi
		c = np.array([0,3])
		for theta in np.linspace(theta_start, theta_end, int(pts/2)):
			p = c + np.array([r*np.sin(theta), -r*np.cos(theta)])
			p = np.append(p, theta)
			pts_all.append(p)
		# plt.plot(np.array(pts_all)[:,0], np.array(pts_all)[:,1])
		return pts_all

	def draw(self):
		# p = PatchCollection(patches, cmap=matplotlib.cm.jet, alpha=0.4)
		p = PatchCollection(self.patches, match_original=True)
		plt.xlim(-5, 5)
		plt.ylim(-5, 5)
		self.ax.add_collection(p)


if __name__ == '__main__':
	f,ax = plt.subplots(1,1)
	T = triangle(f,ax)
	MP = motion_path(f,ax)

	T.add_patches()
	# MP.motion_path()
	T.draw()
	# MP.draw()
	# motion_points = MP.motion_path_pts(100)
	# for p in motion_points:
	# 	print('Move To Pose: %s' %p)
	# 	T.move_triangle(T.triangle_pose, p)
	# 	MP.draw()


	T.move_triangle(T.triangle_pose, [1, 2, np.pi])	
	plt.show()
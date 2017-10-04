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
from moviepy.video.io.bindings import mplfig_to_npimage
import moviepy.editor as mpy

class canvas(object):
	#just does the plotting
	def __init__(self,f,ax):
		self.f, self.ax = f,ax
		self.patches = []

	def getPatches(self, obj):
		self.patches.extend(obj.patches)
		obj.patches = []
	
	def draw(self):
		self.ax.clear()
		p = PatchCollection(self.patches, alpha=0.4, match_original=True)
		self.ax.add_collection(p)
		plt.xlim(-5, 5)
		plt.ylim(-5, 5)
		self.patches = []


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

	def base_triangle(self, pose):
		self.large_vertices_base = self.transform_points(self.large_vertices_base, pose)
		self.small_vertices_base = self.transform_points(self.small_vertices_base, pose)

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

	def pts_to_pose(self, pts):
		pose = [np.append(p,1) for p in pts] #make it transformation matrix friendly
		return np.array(pose)

	def transform_points(self, points, pose):
		points_pose = self.pts_to_pose(points)
		trans = self.transformation_matrix(pose)
		new_pose = [np.dot(trans, p) for p in points_pose]
		new_points = [p[:2] for p in new_pose]
		return new_points

	def large_triangle(self, center_pose):
		self.large_vertices = self.transform_points(self.large_vertices_base, center_pose)
		large_triangle = Polygon(self.large_vertices, True, alpha=0.4)
		self.patches.append(large_triangle)

	def small_triangle(self, center_pose):
		self.small_vertices = self.transform_points(self.small_vertices_base, center_pose)
		small_triangle = Polygon(self.small_vertices, True, alpha=0.4, color = self.small_color)
		self.patches.append(small_triangle)

	def center(self, center_pose):
		center = Circle((center_pose[:2]),radius=0.1, color = self.circle_color, edgecolor = 'auto', alpha=0.4)
		self.patches.append(center)

	def draw(self):
		self.ax.clear()
		p = PatchCollection(self.patches, alpha=0.4, match_original=True)
		self.ax.add_collection(p)
		plt.xlim(-5, 5)
		plt.ylim(-5, 5)
		self.patches = []

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
			p = np.append(p, np.pi + theta)
			pts_all.append(p)
		# plt.plot(np.array(pts_all)[:,0], np.array(pts_all)[:,1])
		return pts_all

	def draw(self):
		# p = PatchCollection(patches, cmap=matplotlib.cm.jet, alpha=0.4)
		p = PatchCollection(self.patches, match_original=True)
		plt.xlim(-5, 5)
		plt.ylim(-5, 5)
		self.ax.add_collection(p)


class makeMovie(object):
	def __init__(self):
		f,ax = plt.subplots(1,1)
		self.T = triangle(f,ax)
		self.path = self.MP = motion_path(f,ax)
		self.path = self.MP.motion_path_pts(100)

	def make_frame_mpl(self,t):
		cur_pose = self.path[int(t*10)]
		self.T.large_triangle(cur_pose)
		self.T.small_triangle(cur_pose)
		self.T.center(cur_pose)
		self.T.draw()
		return mplfig_to_npimage(self.T.f)


if __name__ == '__main__':
	f,ax = plt.subplots(1,1)
	C = canvas(f,ax)
	T = triangle(f,ax)
	T2 = triangle(f,ax)
	T2.triangle_pose = [2,2,np.pi/2]
	T2.base_triangle(T2.triangle_pose)
	
	MP = motion_path(f,ax)
	path = MP.motion_path_pts(100)
	for pose_new in path:
		T.large_triangle(pose_new)
		T2.large_triangle(pose_new)
		T.small_triangle(pose_new)
		T.center(pose_new)
		C.getPatches(T)
		C.getPatches(T2)
		C.draw()
		plt.draw()
		plt.pause(0.001)
	pdb.set_trace()
	T.large_triangle()
	T.draw()
	
	plt.show()



	# MOV = makeMovie()
	# animation = mpy.VideoClip(MOV.make_frame_mpl, duration = 10)
	# pdb.set_trace()
	# animation.write_gif("sinc_mpl.gif", fps = 20)


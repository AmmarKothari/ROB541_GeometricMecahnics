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
import copy
import seaborn as sns
sns.set()

class canvas(object):
	#just does the plotting
	def __init__(self,f,ax):
		self.f, self.ax = f,ax
		self.patches = []

	def getPatches(self, obj, clear=True):
		self.patches.extend(obj.patches)
		if clear: obj.patches = []
	
	def draw(self):
		self.ax.clear()
		p = PatchCollection(self.patches, alpha=0.4, match_original=True)
		self.ax.add_collection(p)
		plt.xlim(-5, 5)
		plt.ylim(-5, 5)
		plt.xlabel('X'); plt.ylabel('Y')
		self.patches = []


class triangle(object):
	# this is the object that will be rotated
	def __init__(self):
		self.patches = []
		self.pose = [0,0,0]
		self.large_vertices = np.array([[-1,-1], [-1, 1], [1, 0]])
		self.small_vertices = np.array([[0,1],[0,-1],[1,0]])

		self.large_vertices_base = np.array([[-1,-1], [-1, 1], [1, 0]])
		### to scale, subtract point to scale around, scale, add point to scale around
		scale_factor = 0.5
		scale_point = self.large_vertices_base[-1]
		self.small_vertices_base = (self.large_vertices_base - scale_point) * [scale_factor, scale_factor] + scale_point

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

	def pose_from_transformation_matrix(self, transformation_matrix):
		pose = np.zeros(3)
		pose[0] = transformation_matrix[0,2]
		pose[1] = transformation_matrix[1,2]
		pose[2] = np.arctan2(transformation_matrix[0,1], transformation_matrix[0,0])
		return pose

	def base_triangle(self, pose):
		self.large_vertices_base = self.transform_points(self.large_vertices_base, pose)
		self.small_vertices_base = self.transform_points(self.small_vertices_base, pose)

	def pts_to_pose(self, pts):
		pose = [np.append(p,1) for p in pts] #make it transformation matrix friendly
		return np.array(pose)

	def transform_points(self, points, pose):
		points_pose = self.pts_to_pose(points)
		trans = self.transformation_matrix(pose)
		new_pose = [np.dot(trans, p) for p in points_pose]
		new_points = [p[:2] for p in new_pose]
		return new_points

	def transform_pose(self, pose1, pose2):
		pts1 = self.pts_to_pose([pose1[:2]])
		new_pts1 = self.transform_points(pts1, pose2)
		pose1_new = new_pts1[0:2] + [pose1[2] + pose2[2]]
		pdb.set_trace()
		return pose1_new

	def large_triangle(self, center_pose):
		self.large_vertices = self.transform_points(self.large_vertices_base, center_pose)
		# self.large_vertices = self.left_action(self.large_vertices_base, center_pose)
		large_triangle = Polygon(self.large_vertices, True, alpha=0.4, color = self.large_color, edgecolor = 'k', linestyle = 'solid', linewidth = 2)
		self.patches.append(large_triangle)

	def small_triangle(self, center_pose):
		self.small_vertices = self.transform_points(self.small_vertices_base, center_pose)
		small_triangle = Polygon(self.small_vertices, True, alpha=0.4, color = self.small_color, edgecolor = 'auto', linestyle = 'solid', linewidth = 2)
		self.patches.append(small_triangle)

	def center(self, center_pose):
		center = Circle((center_pose[:2]),radius=0.1, color = self.circle_color, edgecolor = 'auto', alpha=0.4)
		self.patches.append(center)
		return center

	def left_action(self, current_pose, global_pose):
		combined_transformation = np.dot(self.transformation_matrix(global_pose), self.transformation_matrix(current_pose))
		pose = self.pose_from_transformation_matrix(combined_transformation)
		pose[2] = current_pose[2] + global_pose[2]
		self.pose = pose
		self.large_triangle(pose)
		self.small_triangle(pose)
		c = self.center(pose)
		return c

	def right_action(self, current_pose, relative_pose):
		combined_transformation = np.dot(self.transformation_matrix(current_pose), self.transformation_matrix(relative_pose))
		pose = self.pose_from_transformation_matrix(combined_transformation)
		pose[2] = current_pose[2] + relative_pose[2] # i have no idea why this is here?!? but it seems to work -- is this from the semidirect feature of the group?
		# if so, why doesn't it come straight out of the matrix multiplcation?  Why do I have to adjust it here?
		self.large_triangle(pose)
		self.small_triangle(pose)
		c = self.center(pose)
		return c
		
	def draw(self):
		self.ax.clear()
		p = PatchCollection(self.patches, alpha=0.4, match_original=True)
		self.ax.add_collection(p)
		plt.xlim(-5, 5)
		plt.ylim(-5, 5)
		self.patches = []

class motion_path(object):
	def __init__(self):
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

class track_path(object):
	def __init__(self):
		self.patches = []

	def add_patch(self, patch):
		self.patches.append(patch)

class makeMovie(object):
	def __init__(self):
		f,ax = plt.subplots(1,1)
		self.C = canvas(f,ax)
		self.T = triangle()
		self.T2 = triangle()
		self.path = self.MP = motion_path()
		self.path = self.MP.motion_path_pts(100)
		self.TP = track_path()
		self.TP2 = track_path()

	def single_triangle(self,t):
		cur_pose = self.path[int(t*10)]
		self.T.triangle(cur_pose)
		self.C.getPatches(self.T)
		self.C.draw()
		return mplfig_to_npimage(self.C.f)

	def two_triangle_with_path(self,t):
		h_local = [-2,-1,np.pi/4]
		base_pose = [0,0,0]
		cur_pose = self.path[int(t*10)]
		c_patch = self.T.left_action(base_pose, cur_pose)
		self.TP.add_patch(c_patch)
		#find local pose given new pose
		c_patch = self.T2.right_action(cur_pose, h_local)
		self.TP.add_patch(c_patch)
		self.C.getPatches(self.T)
		self.C.getPatches(self.T2)
		self.C.getPatches(self.TP, clear = False)
		self.C.draw()
		return mplfig_to_npimage(self.C.f)

# class line(object):
# 	#plots line between two centers
# 	def __init__(self):




if __name__ == '__main__':
	# f,ax = plt.subplots(1,1)
	# base_pose = [0,0,0]
	# C = canvas(f,ax)
	# T = triangle()
	# T2 = triangle()
	# TP = track_path()
	# MP = motion_path()
	# h_local = [-2,-2,np.pi/4]
	# path = MP.motion_path_pts(100)
	# for pose_new in path:
	# 	c1 = T.left_action(base_pose, pose_new)
	# 	c2 = T2.right_action(pose_new, h_local)
	# # 	#find local pose given new pose
	# 	TP.add_patch(c1)
	# 	TP.add_patch(c2)
	# 	C.getPatches(T)
	# 	C.getPatches(T2)
	# 	C.getPatches(TP, clear = False)
	# 	C.draw()
	# 	plt.draw()
	# 	plt.pause(0.001)
	# pdb.set_trace()



	MOV = makeMovie()
	animation = mpy.VideoClip(MOV.two_triangle_with_path, duration = 10)
	animation.write_gif("HW1.gif", fps = 20)


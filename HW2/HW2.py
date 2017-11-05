import numpy as np
import pdb
import matplotlib.pyplot as plt
from pprint import pprint
from ROB541_GeometricMecahnics.HW1 import canvas, triangle, motion_path, track_path, makeMovie
from matplotlib.animation import FuncAnimation


pi = np.pi
cos = np.cos
sin = np.sin

TIME_START = 10.0
TIME_DELTA = 0.1
TRIANGLE_SCALE = 0.2
Gb = [1,1,-pi/4]
G1 = [0,0,0]
G2 = [-1,1,pi/2]
G3 = [-1,-1,-pi/4]
G4 = [1,-1,pi]
G_DOT = np.array([1,0,1])

class makeMovie2(makeMovie):
	def __init__(self):
		super(makeMovie2, self).__init__()

	def bodyVelocity(self):
		plt.ion()
		plt.show()
		self.gs = [Gb, G1, G2, G3, G4]
		g_dot = G_DOT
		g_circ_right = self.T.g_circ_right(self.gs[0], g_dot)
		self.triangles = [triangle(g, scale = TRIANGLE_SCALE) for g in self.gs]
		patches = [t.right_action(t.pose, [0,0,0]) for t in self.triangles]
		[self.TP.add_patch(p) for p in patches]
		self.plotBodies()
		self.C.draw()
		plt.draw()
		t_last = 0
		for t in np.arange(0,TIME_START,TIME_DELTA):
			t_delta = t - t_last
			t_last = t
			gs_dot = [np.dot(self.T.transformation_matrix(g_i), g_circ_right) for g_i in self.gs]
			for i,g_i in enumerate(self.gs):
				h = g_circ_right * t_delta
				p = self.triangles[i].right_action(self.triangles[i].pose, h)
				self.TP.add_patch(p)
			self.plotBodies()
			self.C.getPatches(self.TP, clear = False)

	def spatialVelocity(self):
		plt.ion()
		plt.show()
		self.gs = [Gb, G1, G2, G3, G4]
		g_dot = G_DOT
		g_circ_left = self.T.g_circ_left(self.gs[0], g_dot)
		g_circ_right = self.T.g_circ_right(self.gs[0], g_dot)
		self.triangles = [triangle(g, scale = TRIANGLE_SCALE) for g in self.gs]
		patches = [t.right_action(t.pose, [0,0,0]) for t in self.triangles]
		[self.TP.add_patch(p) for p in patches]
		self.plotBodies()
		self.C.draw()
		plt.draw()
		t_last = 0
		for t in np.arange(0,TIME_START,TIME_DELTA):
			t_delta = t - t_last
			t_last = t
			for i,g_i in enumerate(self.gs):
				p = self.triangles[i].left_action(self.triangles[i].pose, g_circ_left * t_delta)
				# g_dot_left = self.triangles[i].g_dot_from_g_circ_left(self.triangles[i].pose, g_circ_left)
				# p = self.triangles[i].move_to_pose(self.triangles[i].pose + g_dot_left.T[0] * t_delta)
				self.TP.add_patch(p)
			self.triangles[i].drawSpatialGeneratorFieldLeft(self.ax, self.triangles[i].pose, g_circ_left * t_delta)
			self.plotBodies()
			self.C.getPatches(self.TP, clear = False)
			self.C.draw()
			plt.draw()
			plt.pause(0.001)
			yield self.f

	def spatialVelocity_animation(self):
		[frame for frame in self.triangle_with_body_velocity()]


	def spatialVelocity_gif(self):
		gen = self.spatialVelocity()
		frame_func = lambda t: next(gen)
		animation = FuncAnimation(self.f, frame_func, frames = np.arange(0,TIME_START,TIME_DELTA), interval = 200)
		animation.save('HW2_supplemental.gif', dpi = 80, writer = 'imagemagick')

	def plotBodies(self):
		[self.C.getPatches(t)  for t in self.triangles]

	def single_triangle_with_velocity(self):
		plt.ion()
		plt.show()
		last_t = -0.1
		base_pose = [0,0,0]
		for t in np.arange(0,TIME_START,TIME_DELTA):
			h_local = [-2,-1,np.pi/4]
			cur_pose = self.path[int(t*10)]
			vel1 = (np.array(self.T.pose) - np.array(self.T.last_pose)) / (t - last_t)
			body_vel1 = self.T.g_circ_right(self.T.pose, vel1)
			c_patch = self.T.left_action(base_pose, cur_pose)
			self.T.drawVelocity(vel1)


			g_circ_left = self.T.g_circ_left(cur_pose, vel1)
			self.T.drawSpatialGeneratorFieldLeft(self.ax, cur_pose, g_circ_left)
			# g_circ_right = self.T.g_circ_right(cur_pose, vel1)
			# self.T.drawSpatialGeneratorFieldRight(self.ax, cur_pose, g_circ_right)

			self.TP.add_patch(c_patch)
			#find local pose given new pose
			c_patch = self.T2.right_action(cur_pose, h_local)
			vel2 = (np.array(self.T2.pose) - np.array(self.T2.last_pose)) / (t - last_t)
			self.T2.drawVelocity(vel2)
			self.TP.add_patch(c_patch)
			self.C.getPatches(self.T)
			self.C.getPatches(self.T2)
			self.C.getPatches(self.TP, clear = False)
			self.C.draw()
			plt.draw()
			plt.pause(0.0001)
			last_t = t
			# print('Vel1: %s, Vel2: %s' %(vel1[2], vel2[2]))
			print('Body Vel1: %s' %body_vel1)

	def triangle_with_body_velocity(self):
		plt.figure(1)
		self.ax = plt.subplot2grid((2,2), (0,0), colspan=2)
		self.ax2 = plt.subplot2grid((2,2), (1,0), colspan=1)
		self.ax3 = plt.subplot2grid((2,2), (1,1), colspan=1)
		self.f = self.ax.get_figure()
		self.C = canvas(self.f,self.ax)
		self.C2 = canvas(self.f, self.ax2)
		self.C3 = canvas(self.f, self.ax3)
		self.C2.set_limits(-1, 1)
		self.C3.set_limits(-1, 1)
		self.T_base1 = triangle()
		self.T_base2 = triangle()
		plt.ion()
		plt.show()
		self.last_t = -0.1
		base_pose = [0,0,0]
		h_local = [-2,-1,np.pi/4]
		for t in np.arange(0,TIME_START,TIME_DELTA):
			cur_pose = self.path[int(t*10)]
			vel1 = (np.array(self.T.pose) - np.array(self.T.last_pose)) / (t - self.last_t)
			body_vel1 = self.T.g_circ_right(self.T.pose, vel1)
			c_patch = self.T.left_action(base_pose, cur_pose)
			self.T.drawVelocity(vel1)

			g_circ_left = self.T.g_circ_left(cur_pose, vel1)
			self.T.drawSpatialGeneratorFieldLeft(self.ax, g_circ_left)
			self.T_base1.move_to_pose([0,0,0])
			self.T_base1.drawVelocity(body_vel1)
			self.TP.add_patch(c_patch)
			#find local pose given new pose
			c_patch = self.T2.right_action(cur_pose, h_local)
			vel2 = (np.array(self.T2.pose) - np.array(self.T2.last_pose)) / (t - self.last_t)
			body_vel2 = self.T2.g_circ_right(self.T2.pose, vel2)
			self.T2.drawVelocity(vel2)
			self.T_base2.move_to_pose([0,0,0])
			self.T_base2.drawVelocity(body_vel2)

			# add patches to plot
			self.TP.add_patch(c_patch)
			self.C.getPatches(self.T)
			self.C.getPatches(self.T2)
			self.C.getPatches(self.TP, clear = False)
			self.C.draw()
			self.C2.getPatches(self.T_base1)
			self.C2.draw()
			self.C3.getPatches(self.T_base2)
			self.C3.draw()
			self.last_t = t
			plt.draw()
			plt.pause(0.0001)
			yield self.f

	def triangle_with_body_velocity_animation(self):
		[frame for frame in self.triangle_with_body_velocity()]


	def triangle_with_body_velocity_gif(self):
		gen = self.triangle_with_body_velocity()
		frame_func = lambda t: next(gen)
		animation = FuncAnimation(self.f, frame_func, frames = np.arange(0,TIME_START,TIME_DELTA), interval = 200)
		animation.save('HW2.gif', dpi = 80, writer = 'imagemagick')

if __name__ == '__main__':
	M = makeMovie2()
	# M.bodyVelocity()
	# M.spatialVelocity()
	# M.spatialVelocity_gif()
	# M.single_triangle_with_velocity()
	M.triangle_with_body_velocity_gif()
	# M.triangle_with_body_velocity_animation()
	

	T = triangle()
	g_dot = [2,2,np.pi/2]
	g = [1,-1,0]
	g_circ_left = [1,0,-1]
	# g_circ_left = T.g_circ_left(g, g_dot)
	# out = T.spatialGeneratorFieldLeft(g, g_circ_left)
	# pdb.set_trace()
	ax = plt.subplot(1,1,1)
	T.drawSpatialGeneratorFieldLeft(ax, g_circ_left)
	plt.show()


	# M.triangle_with_body_velocity_animation()
	# M.single_triangle_with_velocity()

	pdb.set_trace()
import sympy
sympy.init_printing()
import numpy as np
from pprint import pprint
import pdb

def sincos(gamma):
	if isinstance(gamma, float):
		c = np.cos(gamma)
		s = np.sin(gamma)
	else:
		c = sympy.cos(gamma)
		s = sympy.sin(gamma)
	return c,s

def RotX(gamma):
	c,s = sincos(gamma)
	Rx = [
	[1,	0,	0,	0],
	[0,	c,	-s,	0],
	[0,	s,	c,	0],
	[0,	0,	0,	1]
	]
	return np.array(Rx)

def RotY(beta):
	c,s = sincos(beta)
	Ry = [
	[c,		0,	s,	0],
	[0,		1,	0,	0],
	[-s,	0,	c,	0],
	[0,		0,	0,	1]
	]
	return np.array(Ry)

def RotZ(alpha):
	c,s = sincos(alpha)
	Rz = [
	[c,	-s,	0,	0],
	[s,	c,	0,	0],
	[0,	0,	1,	0],
	[0,	0,	0,	1]
	]
	return np.array(Rz)

def Tl(x,y,z):
	T = [
	[1,	0,	0,	x],
	[0,	1,	0,	y],
	[0,	0,	1,	z],
	[0,	0,	0,	1]
	]
	return np.array(T)

def groupRepresentation(ang, xyz=None):
	# this is based on yaw pitch roll 'standard'
	# which is rotation around x and then y and then z
	if xyz is None:
		ang = ang + xyz
	gamma, beta, alpha = ang[:]
	x, y, z = xyz[:]
	Rx = RotX(gamma)
	Ry = RotY(beta)
	Rz = RotZ(alpha)
	t = Tl(x, y, z)
	T = t.dot(Rz).dot(Ry).dot(Rx)
	if all([isinstance(x,float) for x in ang + xyz]):
		T = T.astype(np.float)
	return np.array(T)
	
def rightAction(g, h):
	out = np.dot(g,h)
	if all([isinstance(x,float) for x in out.flatten()[:]]):
		out = out.astype(np.float)
	return out

def leftAction(h, g):
	return np.dot(g,h)

def poseFromMatrix(h):
	if all([isinstance(x, float) for x in h.flatten()[:]]):
		cos = np.cos
		sin = np.sin
		atan2 = np.atan2
	else:
		cos = sympy.cos
		sin = sympy.sin
		atan2 = sympy.atan2
	x = h[0,-1]
	y = h[1,-1]
	z = h[2,-1]
	gamma = atan2(h[2,1],h[2,2])
	alpha = atan2(h[1,0], h[0,0])
	# second term simplifies to cos(beta)
	beta = atan2(-h[2,0], h[0,0]*cos(alpha) + h[1,0]*sin(alpha))

	return np.array([x, y, z, gamma, alpha, beta]).reshape(-1,1)


############################### TESTS ####################################

def RotTest(ang = [np.pi/2, np.pi/2, np.pi/2]):
	print('Gamma: %s' %ang[0])
	pprint('Rotation X:')
	pprint(RotX(ang[0]).round(2))
	gamma = sympy.symbols('gamma')
	print('Gamma: %s' %gamma)
	pprint('Rotation X')
	pprint(RotX(gamma))
	pprint('*'*50)

	print('Beta: %s' %ang[1])
	pprint('Rotation Y:')
	pprint(RotX(ang[1]).round(2))
	beta = sympy.symbols('beta')
	print('Beta: %s' %beta)
	pprint('Rotation Y')
	pprint(RotY(beta))
	pprint('*'*50)

	print('Alpha: %s' %ang[2])
	pprint('Rotation Z:')
	pprint(RotX(ang[2]).round(2))
	alpha = sympy.symbols('alpha')
	print('Alpha: %s' %alpha)
	pprint('Rotation Z')
	pprint(RotZ(beta))
	pprint('*'*50)

def TranslationTest(xyz = [1, 2, 3]):
	x,y,z = xyz[:]
	pprint('Translation')
	pprint('X: %s, Y: %s, Z: %s' %(x, y, z))
	pprint(Tl(x,y,z))
	x,y,z = sympy.symbols('x y z')
	pprint('X: %s, Y: %s, Z: %s' %(x, y, z))
	pprint(Tl(x,y,z))

def TransformationTest(ang = [np.pi/2, np.pi/2, np.pi/2], xyz = [1, 2, 3]):
	pprint('*'*50)
	pprint('Angles: %s' %ang)
	pprint('Translation: %s' %xyz)
	pprint('Transformation')
	out = groupRepresentation(ang, xyz)
	try:
		pprint(out.round(2))
	except:
		pprint(out)

def groupRepresentationTest():
	pprint('*'*50)
	gamma_h, beta_h, alpha_h = sympy.symbols('gamma_h beta_h alpha_h')
	x_h, y_h, z_h = sympy.symbols('x_h y_h z_h')
	h = groupRepresentation(ang = [gamma_h, beta_h, alpha_h], xyz = [x_h, y_h, z_h])
	pprint('h')
	pprint(h)
	pprint('*'*50)

def poseFromGroupTest():
	pprint('*'*50)
	gamma_h, beta_h, alpha_h = sympy.symbols('gamma_h beta_h alpha_h')
	x_h, y_h, z_h = sympy.symbols('x_h y_h z_h')
	h = groupRepresentation(ang = [gamma_h, beta_h, alpha_h], xyz = [x_h, y_h, z_h])
	pprint('Symbolic h')
	pprint(h)
	pprint('Pose')
	pose = poseFromMatrix(h)
	print(pose)
	h = groupRepresentation(ang = [0,0,0], xyz=[0,0,0])
	pprint('Analytic h')
	pprint(h)
	pprint('Pose')
	pose = poseFromMatrix(h)
	print(pose)
	pprint('*'*50)


def groupActionTest():
	pprint('*'*50)
	pprint('Group Operations Analytic')
	g = groupRepresentation(ang = [0,0,0], xyz=[1,0,0])
	h = groupRepresentation(ang = [0,0,np.pi/2], xyz=[0,0,0])

	gh = rightAction(g,h)
	print('Right Action: gh')
	print(gh.astype('float').round(2))

	print('Left Action: hg')
	hg = leftAction(g,h)
	print(hg.astype('float').round(2))


	pprint('*'*50)
	gamma_h, beta_h, alpha_h = sympy.symbols('gamma_h beta_h alpha_h')
	x_h, y_h, z_h = sympy.symbols('x_h y_h z_h')
	gamma_g, beta_g, alpha_g = sympy.symbols('gamma_g beta_g alpha_g')
	x_g, y_g, z_g = sympy.symbols('x_g y_g z_g')
	h = groupRepresentation(ang = [gamma_h, beta_h, alpha_h], xyz = [x_h, y_h, z_h])
	g = groupRepresentation(ang = [gamma_g, beta_g, alpha_g], xyz = [x_g, y_g, z_g])

	gh = rightAction(g,h)
	print('Right Action: gh')
	print(gh)
	pprint('*'*50)

	print('Left Action: hg')
	hg = leftAction(g,h)
	print(hg)
	pdb.set_trace()




def main():
	# RotTest()
	# TranslationTest()
	# TransformationTest(ang = [0,0,np.pi/2], xyz = [1,0,0])

	# poseFromGroupTest()
	groupActionTest()

	# pprint('*'*50)
	# gamma_h, beta_h, alpha_h = sympy.symbols('gamma_h beta_h alpha_h')
	# x_h, y_h, z_h = sympy.symbols('x_h y_h z_h')
	# gamma_g, beta_g, alpha_g = sympy.symbols('gamma_g beta_g alpha_g')
	# x_g, y_g, z_g = sympy.symbols('x_g y_g z_g')
	# h = groupRepresentation(ang = [gamma_h, beta_h, alpha_h], xyz = [x_h, y_h, z_h])
	# g = groupRepresentation(ang = [gamma_g, beta_g, alpha_g], xyz = [x_g, y_g, z_g])
	# pprint('h')
	# pprint(h)
	# pprint('*'*50)
	# pprint('g')
	# pprint(g)
	# pprint('*'*50)

	# gh = rightAction(g,h)
	# print('Right Action: gh')
	# print(gh)
	# pprint('*'*50)

	# print('Left Action: hg')
	# hg = leftAction(g,h)
	# print(hg)
	# pdb.set_trace()
	

if __name__ == '__main__':
	main()
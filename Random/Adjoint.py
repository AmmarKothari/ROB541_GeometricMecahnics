import numpy as np
from pprint import pprint
import pdb

pi = np.pi
def T(x,y,theta):
	T = [[np.cos(theta), -np.sin(theta), x],
		[np.sin(theta), np.cos(theta), y],
		[0,0,1]	]
	return np.array(T)

def inv(T):
	return np.linalg.inv(T)

def get_pose(T):
	x = T[0,2]
	y = T[1,2]
	theta = np.arctan2(T[1,0], T[0,0])
	return np.array([x,y,theta])




g = T(1,1,pi/2)
h1 = T(2,2,pi/2)
h2 = T(2,0,pi/2)

AD_g_h2 = np.dot(np.dot(g,h2),inv(g))
AD_g_h1 = np.dot(np.dot(inv(g),h1),g)
h1_g = np.dot(h1,g)
g_h2 = np.dot(g,h2)
pdb.set_trace()
pprint(g)
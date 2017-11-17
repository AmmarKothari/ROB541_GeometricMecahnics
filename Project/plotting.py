import ThreeDGeoOps as GO
import matplotlib.pyplot as plt





def visualizeRotations():
	pose = [0,0,0, 1, 0, 0]
	g = GO.groupRepresentation(pose)
	h = [0,0,0,0,0,0]
	h_last = [0,0,0,0,0,0]
	for i in np.arange(0, np.pi, 0.1):
		h[2] += i # rotatin around z






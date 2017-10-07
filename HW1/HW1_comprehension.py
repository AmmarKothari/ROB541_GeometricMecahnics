import sympy
import numpy as np
from pprint import pprint
import pdb

class AdditionCanonical(object):
	def __init__(self, N):
		syms = sympy.symbols('a1:%s' %(N+1))
		self.m = sympy.Matrix.eye(2*N-1)
		for i,s in enumerate(syms):
			self.m[1,2*i] = s
		pprint(self.m)

class ScaleShiftCanonical(object):
	def __init__(self):
		syms = sympy.symbols('a1:3')
		self.m = sympy.Matrix.eye(3)
		self.m[0,0] = syms[0]
		self.m[1,2] = syms[1]
		pprint(self.m)


if __name__ == '__main__':
	# t = 3
	# A = AdditionCanonical(t)
	# B = AdditionCanonical(t)
	# pprint(A.m*B.m)

	A = ScaleShiftCanonical()
	B = ScaleShiftCanonical()
	pprint(A.m*B.m)
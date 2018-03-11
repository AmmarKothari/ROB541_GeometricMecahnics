import torch
import torch.nn.functional as F
from torch.autograd import Variable

import scipy.io
import numpy as np
from itertools import count
import matplotlib.pyplot as plt


import pdb

PER_OF_TRAINING = 0.9
BATCH_SIZE = 32
BATCHES = 1
HIDDEN_UNITS_PER_LAYER = 20

mat_var_dict = scipy.io.loadmat('invKinTrainData')

trainData = mat_var_dict['training_data']



def get_batch(train, test, batch_size=BATCH_SIZE):
	idx = np.random.permutation(train.shape[0])
	x = train[idx[:batch_size]]
	y = test[idx[:batch_size]]
	# pdb.set_trace()
	return Variable(torch.from_numpy(x).float()),Variable(torch.from_numpy(y).float())



trainData_ja = np.array([val[:] for val in trainData[:,0]]).reshape(-1,6)
trainData_ee = np.array([val[:] for val in trainData[:,1]]).reshape(-1,6)

trainingNumFiles = int(trainData.shape[0] * PER_OF_TRAINING);
all_idx = np.random.permutation(trainData.shape[0])
train_idx = all_idx[:trainingNumFiles]
test_idx = all_idx[trainingNumFiles:]

train_ja = trainData_ja[train_idx,:]
train_ee = trainData_ja[test_idx,:]

test_ja = trainData_ja[train_idx,:]
test_ee = trainData_ja[test_idx,:]

loss_all = []

# Define model
fc = torch.nn.Linear(train_ee[0].size, train_ja[0].size)
D_in = train_ee[0].size
D_out = train_ja[0].size
x = Variable(torch.randn(BATCH_SIZE, D_in))
y = Variable(torch.randn(BATCH_SIZE, D_out))

model = torch.nn.Sequential(
	torch.nn.Linear(D_in, HIDDEN_UNITS_PER_LAYER),
	torch.nn.ReLU(),
	torch.nn.Linear(HIDDEN_UNITS_PER_LAYER, HIDDEN_UNITS_PER_LAYER),
	torch.nn.ReLU(),
	torch.nn.Linear(HIDDEN_UNITS_PER_LAYER, D_out),
	)

loss_fn = torch.nn.MSELoss(size_average=False)
learning_rate = 1e-4
for batch_idx in count(1): #infinte iterator
	# get Data
	batch_x, batch_y = get_batch(train_ee, train_ja)

	# forward pass
	# output = F.smooth_l1_loss(fc(batch_x), batch_y)
	# loss = output.data[0]
	y_pred = model(batch_x)
	loss = loss_fn(y_pred, batch_y)

	#Reset Gradients
	# fc.zero_grad()
	model.zero_grad()

	# backward pass
	# output.backward()
	loss.backward()

	# Apply gradients
	# for param in fc.parameters():
		# param.data.add_(-0.1 * param.grad.data)
	for param in model.parameters():
		param.data -= learning_rate * param.grad.data



	if batch_idx > 10000:
		break

	# loss_all.append(loss)
	# pdb.set_trace()
	loss_all.append(loss.data.numpy()[0])
	# Stop criterion
	if loss.data.numpy()[0] < 1e-3:
		break

print('Loss: {:.6f} after {} batches'.format(loss.data.numpy()[0], batch_idx))
plt.plot(loss_all)
plt.show()
# print('==> Learned function:\t' + poly_desc(fc.weight.data.view(-1), fc.bias.data))
# print('==> Actual function:\t' + poly_desc(W_target.view(-1), b_target))
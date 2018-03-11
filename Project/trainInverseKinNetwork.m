function trainInverseKinNetwork()

TRAINING_POINTS = 1e3;
TRAINING_DATA_FN = 'invKinTrainData.mat';

% waist
a1 = [0,0,0,0,0,1]; % rotation around z
h1 = [0,0,1,0,0,0]; % extending in z
% shoulder
a2 = [0,0,0,1,0,0]; % rotation around x
h2 = [1,0,0,0,0,0]; % extending in x
% elbow
a3 = [0,0,0,0,1,0]; % rotation around y
h3 = [0,1,0,0,0,0]; % extending in y
% waist
a4 = [0,0,0,0,0,1]; % rotation around z
h4 = [0,0,1,0,0,0]; % extending in z
% shoulder 
a5 = [0,0,0,1,0,0]; % rotation around x
h5 = [1,0,0,0,0,0]; % extending in x
% elbow
a6 = [0,0,0,0,1,0]; % rotation around y
h6 = [0,1,0,0,0,0]; % extending in y

l1 = link(a1, h1,'b', h1/2);
l2 = link(a2, h2,'g', h2/2);
l3 = link(a3, h3,'r', h3/2);
l4 = link(a4, h4,'b', h4/2);
l5 = link(a5, h5,'g', h5/2);
l6 = link(a5, h5,'r', h6/2);
A = arm([l1, l2, l3, l4, l5, l6]);
link_count = length(A.links);
training_data = cell(TRAINING_POINTS, 2);
% first column is the input joint angles
% second column is the resulting poise
if ~(exist(TRAINING_DATA_FN, 'file') == 2)
    for i_tp = 1:TRAINING_POINTS
        % collect data to train network
        ja = rand(link_count,1);
        A_train = A.set_joints(ja);
        A_train = A_train.calc_poses();
        EE = A_train.links(end).distal;
        training_data{i_tp, 1} = ja;
        training_data{i_tp, 2} = EE;
        if rem(i_tp/TRAINING_POINTS,0.1) < 1e-8
            fprintf('%0.2f Training Collection Complete\n', i_tp/TRAINING_POINTS)
        end  
    end
    save(TRAINING_DATA_FN, 'training_data');
else
    load(TRAINING_DATA_FN)
end

% train network
trainingNumFiles = round(size(training_data, 1) * 0.9, 0);
rng(1) % For reproducibility
% [trainDigitData,testDigitData] = splitEachLabel(training_data,trainingNumFiles,'randomize');
idx = [ones(trainingNumFiles, 1); zeros(size(training_data, 1) - trainingNumFiles, 1)];
rand_idx =  idx(randperm(length(idx)));
trainData = training_data(find(rand_idx==1),:);
trainData_y = reshape([trainData{:,1}], [size(trainData,1), length(trainData{1,1})]);
trainData_x = reshape([trainData{:,2}], [size(trainData,1), length(trainData{1,2})]);
testData = training_data(find(rand_idx==0),:);
testData_x = reshape([testData{:,1}], [size(testData,1), length(testData{1,1})]);
net = feedforwardnet(20);
% input EE and get out joint angles
net = configure(net, trainData_x, trainData_y);
% % % pre training
y1 = net(trainData_x);

% % % % train
net = train(net, trainData_x, trainData_y,'useParallel','yes','showResources','yes');
y2 = net(trainData_x);


error = trainData_y - y2;
plot(error, 'rx')
% % % % Compare
% plot(
% layers = [fullyConnectedLayer(10);
%           reluLayer();
%           fullyConnectedLayer(10);
%           reluLayer();
%           fullyConnectedLayer(10);
%           softmaxLayer();
%           ];
% regnet = trainNetwork(trainData,layers,options);
options = trainingOptions('sgdm','MaxEpochs',20,'InitialLearnRate',0.0001);
YTest = classify(regnet,testDigitData);
% A = A.set_joints(start_alphas);
% EE_pose = A.links(end).distal.';


end
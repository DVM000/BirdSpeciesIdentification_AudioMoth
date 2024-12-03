%  Neural Network Training Script

% The script presented here is a modification of the auto-generated code that 
% MATLAB creates when using Deep Learning Toolbox for training a neural network
% It saves the net in finalNeuralNetworkFunction.m

% Steps:
% 1. Load the data and prepare inputs/outputs.
% 2. Divide data into training, validation, and testing subsets.
% 3. Initialize and configure the neural network.
% 4. Train the network and evaluate its performance.
% 5. Save the trained network for deployment.

% Load data for training
dataFile = 'extracted_data.mat'
load(dataFile);

% Initialize inputs and outputs
inputs = zeros(totalInputs, 2 * nbanks);
outputs = zeros(totalInputs, 1);

% Populate inputs and outputs from data
index = 0;
for sampleIdx = 1:size(data, 1)
    for frameIdx = 1:size(data{sampleIdx, 4}, 1) % for each segment
        index = index + 1;
        inputs(index, :) = data{sampleIdx, 4}(frameIdx, :); % MFCC features                   
        outputs(index, 1) = data{sampleIdx, 5}(1);          % Labels (1 for Lesser Kestrel, 0 otherwise)
    end
end


% Divide data into training, validation, and test subsets
% Using a random proportional division of 65% training, 15% validation, and 20% testing
[trainInd,valInd,testInd] = divideint(size(inputs,1),0.65,0.15,0.2);

% Randomly reorder indices within each subset for unbiased training
orderTrainInd = randperm(size(trainInd,2)); 
inputstrain = inputs(trainInd(orderTrainInd),:);
outputstrain = outputs(trainInd(orderTrainInd),:); 

orderTrainVal = randperm(size(valInd,2));
inputsval = inputs(valInd(orderTrainVal),:);
outputsval = outputs(valInd(orderTrainVal),:);

orderTrainTest = randperm(size(testInd,2));
inputstest = inputs(testInd(orderTrainTest),:);
outputstest = outputs(testInd(orderTrainTest),:);

% Combine all subsets into unified input/output arrays for simplicity
inputs = [inputstrain; inputsval; inputstest];
outputs = [outputstrain; outputsval; outputstest];

init(net)

% Prepare input-output for the neural network
% Use selected features for training, excluding some zeroed MFCCs
x = inputs(:,[2:13 43:42+12])'; % Input features (without zeroed MFCC)
t = outputs(:,1)';              % Target labels
t(2,:) = ~t(1,:);				% Create binary target vectors (1-hot encoding)

% Choose a Training Function
trainFcn = 'trainscg';  % Scaled conjugate gradient backpropagation.

% Create a Pattern Recognition Network
hiddenLayerSize = 2;   % Number of hidden neurons
net = patternnet(hiddenLayerSize, trainFcn);  % Initialize network
net.layers{1}.transferFcn = 'tansig';
net.layers{2}.transferFcn = 'softmax';


% Choose Input and Output Pre/Post-Processing Functions
net.input.processFcns = {'removeconstantrows'};
net.output.processFcns = {};

% Setup Division of Data for Training, Validation, Testing
net.divideFcn = 'divideind'; % Partition indices into three sets using specified indices
net.divideParam.trainInd = 1:length(trainInd);
net.divideParam.valInd   = 1+length(trainInd):length(trainInd)+length(valInd);
net.divideParam.testInd  = 1+length(trainInd)+length(valInd):length(trainInd)+length(valInd)+length(testInd);


% Choose a Performance Function
net.performFcn = 'crossentropy';  % Cross-Entropy


% Choose Plot Functions
net.plotFcns = {'plotperform','plottrainstate','ploterrhist', ...
    'plotconfusion', 'plotroc'};

% Train the Network
[net,tr] = train(net,x,t);

% Test the Network
y = net(x);			% Network predictions
e = gsubtract(t,y); % Compute prediction errors
performance = perform(net,t,y) % Overall performance (cross-entropy)

% Compute percentage of misclassified samples
tind = vec2ind(t);  % True labels
yind = vec2ind(y);  % Predicted labels
percentErrors = sum(tind ~= yind)/numel(tind);  % Misclassification rate

% Calculate performance metrics for each data subset
trainPerformance = perform(net,t(tr.trainInd),y(tr.trainInd))  % Training performance
valPerformance = perform(net,t(tr.valInd),y(tr.valInd))        % Validation performance
testPerformance = perform(net,t(tr.testInd),y(tr.testInd))     % Test performance


% View the Network
view(net)

% Deployment
if (true)
    % Generate MATLAB function for neural network for application
    % deployment in MATLAB scripts or with MATLAB Compiler and Builder
    % tools, or simply to examine the calculations your trained neural
    % network performs.
    genFunction(net,'finalNeuralNetworkFunction15khz');
    save('tr') % Save the training record
end

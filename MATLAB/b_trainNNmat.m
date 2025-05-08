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
clear all;

sufix = '_saved'; 
dataFile = strcat('extracted_data',sufix,'.mat');
load(dataFile);

% Display basic information
disp(['Total Inputs: ', num2str(totalInputs)]);
disp(['Total Duration (s): ', num2str(sum(totalDuration))]);
disp(['Number of Inputs per Class: ', num2str(sum(ninputs))]);

% Load pre-split datasets
load(strcat('train_data',sufix,'.mat')); % Train set
load(strcat('val_data',sufix,'.mat'));   % Validation set
load(strcat('test_data',sufix,'.mat'));  % Test set

% Extract MFCC features and labels for each dataset
function [X, Y] = extract_features_labels(data, totalInputs, nbanks)
    X = zeros(totalInputs, 2 * nbanks);
    Y = zeros(totalInputs, 1);
    
    % Populate inputs X and outputs Y from data
    index = 0;
    for sampleIdx = 1:size(data, 1)
        for frameIdx = 1:size(data{sampleIdx, 4}, 1) % For each segment
            index = index + 1;
            X(index, :) = data{sampleIdx, 4}(frameIdx, :); % MFCC features
            Y(index, 1) = data{sampleIdx, 5}(1);          % Labels (1 for Lesser Kestrel, 0 otherwise)
 
        end
    end

    % Data balance
    idx1 = find(Y==1);
    idx0 = find(Y==0);
    if size(idx1,1)<size(idx0,1)
        idx0_random = idx0(randperm(size(idx0,1), size(idx1,1)));
        idx = [idx1; idx0_random];        
    else
        idx = [idx1(1:size(idx0,1)); idx0];
    end
    X = X(idx, :);
    Y = Y(idx, :);

    % Randomly reorder indices within each subset for unbiased training
    orderInd = randperm(size(X,1)); 
    X = X(orderInd,:);
    Y = Y(orderInd,:); 
end

% Extract features and labels
[inputsTrain, outputsTrain] = extract_features_labels(train_data, sum(ninputs(1,:)), nbanks);
[inputsVal, outputsVal] = extract_features_labels(val_data, sum(ninputs(3,:)), nbanks); 
[inputsTest, outputsTest] = extract_features_labels(test_data, sum(ninputs(2,:)), nbanks);

Ntrain = size(inputsTrain,1);
Nval = size(inputsVal,1);
Ntest = size(inputsTest,1);
disp(['Training data: ', num2str(Ntrain)]);
disp([' - (un)balanced as: ', num2str(sum(outputsTrain)), ' ', num2str(Ntrain-sum(outputsTrain))]);
disp(['Validation data: ', num2str(Nval)]);
disp([' - (un)balanced as: ', num2str(sum(outputsVal)), ' ', num2str(Nval-sum(outputsVal))]);
disp(['Test data: ', num2str(Ntest)]);
disp([' - (un)balanced as: ', num2str(sum(outputsTest)), ' ', num2str(Ntest-sum(outputsTest))]);


% Prepare input-output for the neural network
% Use selected features for training, excluding some zeroed MFCCs
inputs = [inputsTrain; inputsVal; inputsTest]; % Combine all inputs
outputs = [outputsTrain; outputsVal; outputsTest]; % Combine all outputs
x = inputs(:,[2:13 43:42+12])'; % Input features (revlevant features without zeroed MFCC)
t = outputs';                   % Target labels. Transpose for NN format
t(2,:) = ~t(1,:);               % Create binary target vectors (1-hot encoding)

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
net.divideParam.trainInd = 1:Ntrain;
net.divideParam.valInd   = 1+Ntrain:Ntrain+Nval;
net.divideParam.testInd  = 1+Ntrain+Nval:Ntrain+Nval+Ntest;

% Choose a Performance Function
net.performFcn = 'crossentropy';  % Cross-Entropy

%https://es.mathworks.com/help/deeplearning/ug/train-and-apply-multilayer-neural-networks.html
net.trainParam.min_grad = 1e-11;
net.trainParam.max_fail = 20;

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
percentErrors = sum(tind ~= yind)/numel(tind)  % Misclassification rate
perrTrain = sum(tind(1:Ntrain) ~= yind(1:Ntrain))/numel(tind(1:Ntrain))
perrVal = sum(tind(1+Ntrain:Ntrain+Nval) ~= yind(1+Ntrain:Ntrain+Nval))/numel(tind(1+Ntrain:Ntrain+Nval))
perrTest = sum(tind(1+Ntrain+Nval:Ntrain+Nval+Ntest) ~= yind(1+Ntrain+Nval:Ntrain+Nval+Ntest))/numel(tind(1+Ntrain+Nval:Ntrain+Nval+Ntest))

ERROR_TRAIN = tind(1:Ntrain) ~= yind(1:Ntrain);
ERROR_VAL = tind(1+Ntrain:Ntrain+Nval) ~= yind(1+Ntrain:Ntrain+Nval);
ERROR_TEST=tind(1+Ntrain+Nval:Ntrain+Nval+Ntest) ~= yind(1+Ntrain+Nval:Ntrain+Nval+Ntest);
%figure; bar(ERROR_TEST);

% Calculate performance metrics for each data subset
trainPerformance = perform(net,t(tr.trainInd),y(tr.trainInd));  % Training performance
valPerformance = perform(net,t(tr.valInd),y(tr.valInd));        % Validation performance
testPerformance = perform(net,t(tr.testInd),y(tr.testInd));     % Test performance

% View the Network
view(net)

% Deployment
if (true)
    % Generate MATLAB function for neural network for application
    % deployment in MATLAB scripts or with MATLAB Compiler and Builder
    % tools, or simply to examine the calculations your trained neural
    % network performs.
    genFunction(net,'NeuralNetworkFunction');
    save('tr') % Save the training record
end

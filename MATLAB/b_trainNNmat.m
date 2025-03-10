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
dataFile = 'extracted_data.mat';
load(dataFile);

% Display basic information
disp(['Total Inputs: ', num2str(totalInputs)]);
disp(['Total Duration (s): ', num2str(sum(totalDuration))]);
disp(['Number of Inputs per Class: ', num2str(sum(ninputs))]);


% Load pre-split datasets
load('train_data.mat'); % Train set
load('val_data.mat');   % Validation set
load('test_data.mat');  % Test set

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
end

% Extract features and labels
[inputsTrain, outputsTrain] = extract_features_labels(train_data, sum(ninputs(1,:)), nbanks);
[inputsVal, outputsVal] = extract_features_labels(val_data, sum(ninputs(2,:)), nbanks);
[inputsTest, outputsTest] = extract_features_labels(test_data, sum(ninputs(3,:)), nbanks);

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
net.divideParam.trainInd = 1:size(inputsTrain,1);
net.divideParam.valInd   = 1+size(inputsTrain,1):size(inputsTrain,1)+size(inputsVal,1);
net.divideParam.testInd  = 1+size(inputsTrain,1)+size(inputsVal,1):size(inputsTrain,1)+size(inputsVal,1)+size(inputsTest,1);


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

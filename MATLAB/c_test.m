%  Neural Network Testing Script

% A) Evaluate at file level. Steps:
%   1. Load the data from complete files and prepare inputs/outputs.
%   2. Evaluate network performance.
%
% B) Evaluate at 32-ms interval level. Steps:
%   1. Load the data from segments and prepare inputs/outputs.
%   2. Evaluate network performance.


% INPUT SETUP
clear all;   % Clear workspace variables

NNfunction = @(x) NeuralNetworkFunction(x)  % Trained Neural Network
TH = 0.5                                    % detection threshold (0.5 by default)

% -------------------------------------------------------------------------
%% TEST AT FILE LEVEL
%--------------------------------------------------------------------------
baseDir = 'TestAudios'; % Base directory
categories = {'presence', 'absence'};

% Ground truth:
SEG_GT = [17, 10, 1, 4, 2, 17, 4, 10, 21, 41, 18, 14, 14, 7, 10, 15, 4, 6, 21, 18, 6, 34, 11, 4, 20, 2, 4, 11, 6, 8, 4, 4, 1, 15, 2, 8, 11, 13, zeros(1, 82)];
% NN detections:
SEG_DET = [];
TP = 0; TN = 0; FP = 0; FN = 0;
files = [];
all_y = [];

function [y, y_pred, Ndet] = predict_over_one_file(filename, TH, NNfunction)
    % Function to calculate possitive predictions for one audio file.
    %  Returns:
    %     y:     logical vector of possitive detections
    %     y_pred: logical output: true if at least one possitive detection
    %     Ndet:   number of possitive predictions

    nbanks = 41;    % Number of mel filter banks for MFCC calculation
    lwindow = 1024; % Window length for signal processing
    fs = 32000;     % Target sampling frequency in Hz

    % Load audio file (handles MP3 and WAV)
    [audio, FS] = audioread(filename);

    % Ensure single-channel processing (convert stereo to mono if necessary)
    if size(audio, 2) > 1
        audio = mean(audio, 2); % Convert stereo to mono by averaging channels
    end

    % Resample audio if necessary
    [P, Q] = rat(fs / FS);  % Calculate resampling ratio to match target sampling frequency (fs)
    resampledAudio = resample(audio, P, Q); % Resample to target frequency

    % Process if length is sufficient
    if length(resampledAudio) > lwindow
        % Calculate MFCC coefficients and their deltas using mfccs.m
        twindow = 1024/32000; % Temporal window length
        [mfccCoeffs, mfccDeltas] = mfccs(resampledAudio', nbanks, twindow, fs, 1e10);

        % Feeding the MFCC coefficients into the neural network for classification
        [y] = NNfunction([mfccCoeffs(:, 2:13) mfccDeltas(:, 2:13)]'); % Feed MFCCs into the network
        y1 = y(1,:);  % Use the output from the neural network (first row, which is the predicted positive class probabilities)
        y = (y1 > TH)';
        y_pred = double(sum(y1 > TH) > 0); % Logical vector of possitive predictions
        Ndet = sum(y1 > TH);               % Number of possitive predictions
    else
        y_pred = -1;
        Ndet = -1;
        y1 = [];
    end
end


% Proccess all testing files
for catIdx = 1:length(categories) % Process each category (Presence/Absence)
    category = categories{catIdx};
    fileLabel = catIdx; % Label index for Presence = 1, Absence = 2
    
    filedir = fullfile(baseDir, category);
    fprintf('- Processing category %s (label %d)\n', category, catIdx-1);
           
     % Find all MP3 and WAV files in the directory
     matfiles = [dir(fullfile(filedir, '*.mp3')); dir(fullfile(filedir, '*.wav')); dir(fullfile(filedir, '*.WAV'))];
     nfilesb = length(matfiles); % Number of audio files found
     if nfilesb == 0
            fprintf('No files found in %s\n', filedir);
            continue;
     end
       
     fileIndex = 1;
     processingComplete = false;

      % Process each audio file in the directory
      while ~processingComplete

        % Read the current audio file
        filename = matfiles(fileIndex).name;
        %fprintf('Processing file %s\n', filename);
        files = [files; filename];
    
        % Calculate predictions
        [y, ypred, Ndet] = predict_over_one_file( fullfile(filedir, filename), TH, NNfunction );
        SEG_DET = [SEG_DET; Ndet];
        all_y = [all_y, y];

        if ypred == -1 % no enough signal lenght
            fileIndex = fileIndex + 1;
            continue;
        end

        % Calculate metrics
        if catIdx == 1 && ypred
            TP = TP + 1;
            fprintf('%s TP. #Detections: %d\n', fullfile(filedir, filename), Ndet);
        elseif catIdx == 1 && ~ypred
            FN = FN + 1;
            fprintf('%s FN. #Detections:  %d\n', fullfile(filedir, filename), Ndet);
        elseif catIdx == 2 && ~ypred
            TN = TN + 1;
            fprintf('%s TN. #Detections:  %d\n', fullfile(filedir, filename), Ndet);
        else
            FP = FP + 1;
            %disp([fullfile(filedir, filename), ' FP ', Ndet]);
            fprintf('%s FP. #Detections:  %d\n', fullfile(filedir, filename), Ndet);
        end

        % Check if all files have been processed
        if fileIndex == length(matfiles)
            processingComplete = true;
        else
            fileIndex = fileIndex + 1; % Move to the next file
        end
      end
        
      fprintf('   processed %d files\n', nfilesb);
end

% Display metrics:
fprintf('TP %d, TN %d, FP %d, FN %d\n', TP, TN, FP ,FN);
acc = (TP+TN)/(TP+TN+FP+FN)
precision = TP/(TP+FP)
recall = TP/(TP+FN)
f1 = 2*precision*recall/(precision+recall)

R2 = corrcoef(SEG_GT, SEG_DET);
R2 = R2(1,2)

% Save predictions
results = [files, strcat(',',num2str(SEG_GT')), strcat(',',num2str(SEG_DET))];
writematrix(results, 'results.csv')

save('predictions_audio.mat', 'all_y', 'files');
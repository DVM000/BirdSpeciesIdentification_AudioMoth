% MATLAB Script for Extracting MFCC Characteristics of Bird Audio Recordings
% This script processes audio recordings (downloaded from Xeno-Canto dataset)
% to extract Mel-Frequency Cepstral Coefficients (MFCCs) as input features
% and assigns labels (0/1) to indicate if the audio belongs to a target 
% bird species: Lesser Kestrel.
% The extracted data is saved for later use in machine learning training.

%% INPUT SETUP
clear all;   % Clear workspace variables
nbanks = 41; % Number of mel filter banks for MFCC calculation
lwindow = 1024; % Window length for signal processing
fs = 32000;     % Target sampling frequency in Hz

% Initialize data storage
data = {};

% Define dataset directories
baseDir = 'dataset'; % Base directory
categories = {'Presence', 'Absence'};

%baseDir = 'datasetv2'; % Base directory
%categories = {'Presence_10min_proporciones', 'Absence9_10min'};
subfolders = {'train', 'test', 'val'};

% To save data
sufix = '_saved';

% Initialize counters
totalDuration = zeros(1, 2); % Two classes: Presence (1) and Absence (0)
ninputs = zeros(3, 2);
current_path = pwd;

N = length(categories); % Number of classes (Presence and Absence)

%% PROCESS EACH CATEGORY (Presence/Absence)
for catIdx = 1:length(categories)
    category = categories{catIdx};
    fileLabel = catIdx; % Label index for Presence = 1, Absence = 2
    
	% Create one-hot encoded label vector
    label = zeros(1, N);
    label(fileLabel) = 1;
	
    for subIdx = 1:length(subfolders)
        subfolder = subfolders{subIdx};  % 'train', 'test', or 'val'
        filedir = fullfile(baseDir, category, subfolder);
        fprintf('- Processing category %s (label %d), subset %s\n', category, catIdx-1, subfolder);
        
        if ~isfolder(filedir)
            fprintf('Skipping missing directory: %s\n', filedir);
            continue;
        end
        
        % Find all MP3 and WAV files in the directory
        matfiles = [dir(fullfile(filedir, '*.mp3')); dir(fullfile(filedir, '*.wav'))];
        nfilesb = length(matfiles); % Number of audio files found

        if nfilesb == 0
            fprintf('No files found in %s\n', filedir);
            continue;
        end
   
        % Call external script for data extraction
        cd(current_path);
        data_extract;

        % Append extracted data
        data = [data; bird]; 

        % Store total duration and input count
        totalDuration(catIdx) = totalDuration(catIdx) + trecord;
        ninputs(subIdx, catIdx) = ninputsloc;
        fprintf('   processed %d files with %d inputs\n', nfilesb, ninputsloc);
    end
end

% Summarize total number of inputs
totalInputs = sum(ninputs);

%% SAVE EXTRACTED DATA
save(strcat('extracted_data',sufix,'.mat'), 'fs', 'nbanks', 'totalInputs', 'totalDuration', 'ninputs');

fprintf('Data extraction complete. Total Inputs: %d\n', totalInputs);
ninputs

% Extract separate datasets based on the stored dataset type
train_data = data(strcmp({data{:,7}}, 'train'), :);
val_data = data(strcmp({data{:,7}}, 'val'), :);
test_data = data(strcmp({data{:,7}}, 'test'), :);

% Save them separately for easier use later
save(strcat('train_data',sufix,'.mat'), 'train_data');
save(strcat('val_data',sufix,'.mat'), 'val_data');
save(strcat('test_data',sufix,'.mat'), 'test_data');

